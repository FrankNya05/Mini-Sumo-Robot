#pragma once
#include <Arduino.h>
#include <math.h>
#include "RobotConstants.hpp"

// ═══════════════════════════════════════════════════════════════
//  EKF.hpp — Filtre de Kalman Étendu pour localisation 2D
//
//  État estimé : [x (mm), y (mm), θ (rad), v (mm/s)]
//
//  Sources de mesure fusionnées :
//  - Accéléromètre (axe longitudinal)     → prédiction de v
//  - Encodeurs (Δθ)                       → prédiction de θ
//  - Encodeurs (vitesse d/dt)             → correction de v (avec rejet si patinage)
//  - IMU gx (gyroscope X = axe lacet)     → correction du cap
//
//  Pourquoi v dans l'état ?
//  Avant, x/y étaient intégrés directement depuis les encodeurs : si les
//  roues patinaient (poussée contre l'adversaire), la position partait à
//  la dérive sans que la covariance le signale. Maintenant les encodeurs
//  sont une MESURE de v, confrontée à la prédiction de l'accéléromètre.
//  Si l'écart est statistiquement improbable (test de Mahalanobis), la
//  mesure est rejetée → le filtre suit l'accéléromètre pendant le patinage.
//
//  Paramètres robot (depuis RobotConstants.hpp) :
//  - Entraxe          : WHEELBASE_MM
//
//  Usage (à chaque cycle, dans cet ordre) :
//      ekf.predict(dl, dr, accelFwdMmS2, dtSec);  // accéléro + Δθ encodeurs
//      ekf.updateOdometry(dl, dr, dtSec);         // vitesse encodeurs
//      ekf.updateIMU(gx, dtSec);                  // gyroscope
//      EKF::Pose pose = ekf.getPose();
// ═══════════════════════════════════════════════════════════════

class EKF {
public:

    static constexpr int N = 4;
    enum Idx { IX = 0, IY = 1, ITH = 2, IV = 3 };

    struct Pose {
        float x     = 0.0f;  // mm
        float y     = 0.0f;  // mm
        float theta = 0.0f;  // rad
    };

    EKF() { reset(); }

    // ── Remise à zéro ─────────────────────────────────────────
    void reset() {
        _x = _y = _theta = _thetaPrev = _v = 0.0f;
        _slipCount = 0;
        _slipping  = false;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) _cov[i][j] = 0.0f;
        _cov[IX][IX]   = 1.0f;
        _cov[IY][IY]   = 1.0f;
        _cov[ITH][ITH] = 0.1f;
        _cov[IV][IV]   = 1.0f;   // robot posé immobile au reset
    }

    // ── Prédiction — accéléromètre + Δθ encodeurs ─────────────
    //  Appeler à chaque cycle taskEncoders (200 Hz)
    //  @param dl           distance roue gauche en mm (signée) — sert au Δθ
    //  @param dr           distance roue droite en mm (signée) — sert au Δθ
    //  @param accelMmS2    accélération longitudinale en mm/s² (avant > 0)
    //  @param dtSec        intervalle de temps en secondes
    void predict(float dl, float dr, float accelMmS2, float dtSec) {
        const float dTheta   = (dr - dl) / WHEELBASE;
        const float midTheta = _theta + dTheta * 0.5f;
        // Distance parcourue selon le modèle cinématique (v constant + a)
        const float d        = _v * dtSec + 0.5f * accelMmS2 * dtSec * dtSec;
        const float c = cosf(midTheta), s = sinf(midTheta);

        _thetaPrev = _theta;
        _x        += d * c;
        _y        += d * s;
        _theta     = _wrapAngle(_theta + dTheta);
        _v        += accelMmS2 * dtSec;

        // Jacobien F = ∂f/∂état
        //   ∂x/∂θ = -d·sin   ∂x/∂v = dt·cos
        //   ∂y/∂θ =  d·cos   ∂y/∂v = dt·sin
        float F[N][N] = {
            { 1.0f, 0.0f, -d * s, dtSec * c },
            { 0.0f, 1.0f,  d * c, dtSec * s },
            { 0.0f, 0.0f,  1.0f,  0.0f      },
            { 0.0f, 0.0f,  0.0f,  1.0f      }
        };

        // Bruit de processus Q
        //  - x,y : erreur de modèle proportionnelle au mouvement (comme avant)
        //  - θ   : erreur encodeurs proportionnelle à la rotation (comme avant)
        //  - v   : bruit accéléromètre intégré sur dt → σa²·dt²
        const float absD = fabsf(d) + fabsf(dTheta) * WHEELBASE * 0.5f;
        float Q[N][N] = {};
        Q[IX][IX]   = _qXY * absD;
        Q[IY][IY]   = _qXY * absD;
        Q[ITH][ITH] = _qTheta * fabsf(dTheta) + 1e-6f;
        Q[IV][IV]   = _qAccel * dtSec * dtSec;

        // P = F * P * F^T + Q
        float FP[N][N], FPFT[N][N];
        _matMul(F, _cov, FP);
        _matMulT(FP, F, FPFT);
        _matAdd(FPFT, Q, _cov);
    }

    // ── Mise à jour — vitesse mesurée par les encodeurs ───────
    //  z = (dl+dr)/2 / dt,  H = [0, 0, 0, 1]
    //
    //  Test de cohérence (gating) : la distance de Mahalanobis
    //      m² = innovation² / S
    //  suit une loi χ² à 1 degré de liberté si la mesure est
    //  cohérente avec le modèle. m² > _gateChi2 (9 = 3σ) → la mesure est
    //  très improbable → on suppose un patinage et on la rejette.
    //
    //  Garde-fou : si on rejette plus de _maxSlipCycles d'affilée,
    //  c'est peut-être l'accéléromètre qui a dérivé (biais) et non les
    //  roues qui patinent → on force l'acceptation pour éviter la
    //  divergence (le filtre ne se « rattraperait » jamais sinon).
    void updateOdometry(float dl, float dr, float dtSec) {
        if (dtSec <= 0.0f) return;
        const float zV         = (dl + dr) * 0.5f / dtSec;
        const float innovation = zV - _v;
        const float S          = _cov[IV][IV] + _rEnc;
        const float mahal2     = innovation * innovation / S;

        if (mahal2 > _gateChi2 && _slipCount < _maxSlipCycles) {
            _slipCount++;
            _slipping = true;
            return;                                   // mesure rejetée
        }
        _slipCount = 0;
        _slipping  = false;

        // K = P H^T / S = colonne v de P / S
        // Contrairement au gyro, on corrige AUSSI x, y : c'est v qui a
        // produit x, y dans predict(), la covariance croisée P[x][v]
        // dit exactement de combien les corriger.
        float K[N];
        for (int i = 0; i < N; i++) K[i] = _cov[i][IV] / S;

        _x     += K[IX]  * innovation;
        _y     += K[IY]  * innovation;
        _theta  = _wrapAngle(_theta + K[ITH] * innovation);
        _v     += K[IV]  * innovation;

        // P = (I - K H) P  → P[i][j] -= K[i] * P[v][j]
        float rowV[N];
        for (int j = 0; j < N; j++) rowV[j] = _cov[IV][j];
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) _cov[i][j] -= K[i] * rowV[j];

        _symmetrize();
    }

    // ── Mise à jour — gyroscope IMU ───────────────────────────
    //  Appeler à chaque cycle taskEncoder (200 Hz)
    //  @param gyroRate  vitesse angulaire gx (lacet) en rad/s — signe : CCW > 0
    //  @param dtSec     intervalle de temps en secondes
    void updateIMU(float gyroRate, float dtSec) {
        const float zMeasured  = gyroRate * dtSec;
        const float dThetaEnc  = _theta - _thetaPrev;
        const float innovation = _wrapAngle(zMeasured - dThetaEnc);

        const float S  = _cov[ITH][ITH] + _rIMU;
        const float K2 = _cov[ITH][ITH] / S;

        // Le gyro mesure uniquement l'angle — K[x]=K[y]=K[v]=0 : seul θ est corrigé.
        // Corriger x,y avec une mesure angulaire polluerait la position quand
        // les covariances croisées cov[x][θ]/cov[y][θ] deviennent non nulles.
        _theta = _wrapAngle(_theta + K2 * innovation);

        // P = (I - KH)P avec K=[0,0,K2,0], H=[0,0,1,0] → seule la ligne θ change
        const float s = 1.0f - K2;
        for (int j = 0; j < N; j++) _cov[ITH][j] *= s;
        // Restaure la symétrie : colonne θ = transposée de la ligne θ
        for (int i = 0; i < N; i++) _cov[i][ITH] = _cov[ITH][i];

        _symmetrize();
    }

    // ── ZUPT (Zero-Velocity UPdaTe) ───────────────────────────
    //  Appeler quand on SAIT que le robot est immobile (roues arrêtées
    //  et consigne nulle depuis un moment). v = 0 est alors une mesure
    //  quasi parfaite : on l'impose et on coupe ses corrélations avec
    //  x, y, θ — sinon un biais accéléro intégré dans v continue de
    //  pousser x, y alors que le robot ne bouge pas.
    void zeroVelocity() {
        _v = 0.0f;
        for (int j = 0; j < N; j++) {
            _cov[IV][j] = 0.0f;
            _cov[j][IV] = 0.0f;
        }
        _cov[IV][IV] = ZUPT_VAR_V;
        _slipCount   = 0;
        _slipping    = false;
    }

    // ── Accesseurs ────────────────────────────────────────────
    Pose  getPose()             const { return { _x, _y, _theta }; }
    float getX()                const { return _x; }
    float getY()                const { return _y; }
    float getTheta()            const { return _theta; }
    float getVelocity()         const { return _v; }          // mm/s
    bool  isSlipping()          const { return _slipping; }
    float getUncertaintyX()     const { return _cov[IX][IX]; }
    float getUncertaintyY()     const { return _cov[IY][IY]; }
    float getUncertaintyTheta() const { return _cov[ITH][ITH]; }
    float getUncertaintyV()     const { return _cov[IV][IV]; }

    // ── Réglage des bruits (à calibrer selon le robot réel) ───
    // Confiance relative Q/R : baisser _rIMU OU monter _qXY/_qTheta a le
    // même effet (plus de poids au gyro face à l'odométrie encodeurs), et
    // inversement — ce n'est pas une confiance absolue mais un rapport.
    void setProcessNoise(float qXY, float qTheta) { _qXY = qXY; _qTheta = qTheta; }
    void setMeasurementNoiseIMU(float rIMU)        { _rIMU = rIMU; }
    void setAccelNoise(float qAccel)               { _qAccel = qAccel; }
    void setEncoderVelocityNoise(float rEnc)       { _rEnc = rEnc; }
    void setSlipGate(float chi2, int maxCycles)    { _gateChi2 = chi2; _maxSlipCycles = maxCycles; }

private:

    static constexpr float ZUPT_VAR_V = 1.0f;   // (mm/s)² — incertitude sur v après ZUPT

    float _x = 0.0f, _y = 0.0f, _theta = 0.0f, _thetaPrev = 0.0f, _v = 0.0f;
    float _cov[N][N] = {};   // matrice de covariance (evite le conflit avec le macro _P de newlib)

    // Bruit processus : confiance en la PRÉDICTION (odométrie encodeurs)
    // entre deux mesures. Baisser _qXY/_qTheta = plus confiance aux
    // encodeurs = le filtre laisse moins les mesures (gyro) le corriger.
    float _qXY    = 0.021f;  // bruit processus position  (mm²/mm parcouru) — calibré test droit: cross-track var=11.2mm² / 540mm
    float _qTheta = 0.1f;    // bruit processus cap       (rad²/rad tourné)
    // Bruit mesure : confiance en la MESURE gyroscope. Baisser _rIMU =
    // plus confiance au gyro = correction plus forte de θ à chaque cycle
    // (voir K2 = cov[θ][θ] / (cov[θ][θ] + _rIMU) dans updateIMU()).
    float _rIMU   = 1e-4f;   // bruit mesure gyroscope (rad²) — calibré: K≈85% pendant virage rapide

    // ── Fusion accéléromètre — NON CALIBRÉ ────────────────────
    // _qAccel : variance de l'accélération longitudinale (mm/s²)².
    //   Inclut bruit capteur + vibrations moteurs + fuite de gravité
    //   due au tangage. Point de départ : σa ≈ 300 mm/s² (0.03 g).
    //   Calibrer : enregistrer a_imu robot immobile moteurs ON → variance.
    float _qAccel = 300.0f * 300.0f;
    // _rEnc : variance de la vitesse encodeurs (mm/s)². 1 pulse ≈ 0.065 mm
    //   sur 5 ms → quantification ≈ 13 mm/s → σ ≈ 13/√12 ≈ 3.8 mm/s.
    //   Arrondi à 5 mm/s pour couvrir le jeu mécanique.
    float _rEnc   = 25.0f;
    // Gating χ² (1 ddl) : 9 → 3σ (≈0.3 % de faux rejets si tout va bien)
    float _gateChi2      = 9.0f;
    int   _maxSlipCycles = 40;   // 40 × 5 ms = 200 ms max de suivi accéléro seul
    int   _slipCount     = 0;
    bool  _slipping      = false;

    static constexpr float WHEELBASE = RobotConstants::WHEELBASE_MM;

    static float _wrapAngle(float a) {
        while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
        while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
        return a;
    }

    // Symétrie numérique globale (accumulation d'erreurs float sur toutes les itérations)
    void _symmetrize() {
        for (int i = 0; i < N; i++)
            for (int j = i + 1; j < N; j++) {
                float avg = (_cov[i][j] + _cov[j][i]) * 0.5f;
                _cov[i][j] = _cov[j][i] = avg;
            }
    }

    static void _matMul(const float A[N][N], const float B[N][N], float C[N][N]) {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) {
                C[i][j] = 0.0f;
                for (int k = 0; k < N; k++) C[i][j] += A[i][k] * B[k][j];
            }
    }

    static void _matMulT(const float A[N][N], const float B[N][N], float C[N][N]) {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) {
                C[i][j] = 0.0f;
                for (int k = 0; k < N; k++) C[i][j] += A[i][k] * B[j][k];
            }
    }

    static void _matAdd(const float A[N][N], const float B[N][N], float C[N][N]) {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) C[i][j] = A[i][j] + B[i][j];
    }
};
