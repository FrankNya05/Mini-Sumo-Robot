#pragma once
#include <cmath>
#include "StrategyInterface.hpp"
#include "RobotContext.hpp"

// ═══════════════════════════════════════════════════════════════
//  SquareStrategy — Suivi de bord + virage 90° (capteurs avant)
//
//  Principe :
//    Le carré est tracé au sol (bande blanche). En avance normale,
//    le capteur FRONT_RIGHT corrige en continu (bang-bang) pour
//    rester sur le bord blanc — corrige la position réelle sur la
//    bande, pas juste le cap (le LQR seul ne suffisait pas : il
//    tient une direction fixe mais ne recentre pas sur la ligne).
//    Dès que les DEUX capteurs avant détectent le blanc à la fois
//    (coin du carré), le robot pivote 90° vers la gauche sur place,
//    suivi précisément par l'EKF θ, puis reprend le suivi de bord.
// ═══════════════════════════════════════════════════════════════

class SquareStrategy : public StrategyInterface {
public:
    const char* name()     const override { return "Square"; }
    LedColor    color()    const override { return LedColor::CYAN; }
    bool        needsLQR() const override { return true; }  // tient le cap pendant le virage

    void reset() override {
        _turning        = false;
        _turnAngleSaved = false;
    }

    RobotConstants::ActionCommand execute(RobotContext& ctx) override {
        using AC = RobotConstants::ActionCommand;

        ctx.setState(RobotConstants::State::ATTACK);

        // ── Virage 90° en cours — suivi EKF θ (delta signé, pas fabsf :
        // le sens de rotation est fixe donc le bruit EKF s'annule au lieu
        // de gonfler le total accumulé — cf. bug trouvé sur Berserker) ──
        if (_turning) {
            if (!_turnAngleSaved) {
                _turnPrevTheta  = ctx.getPose().theta;
                _turnAccum      = 0.0f;
                _turnAngleSaved = true;
            } else {
                float dTheta = ctx.getPose().theta - _turnPrevTheta;
                while (dTheta >  (float)M_PI) dTheta -= 2.0f * (float)M_PI;
                while (dTheta < -(float)M_PI) dTheta += 2.0f * (float)M_PI;
                _turnAccum     += dTheta;
                _turnPrevTheta  = ctx.getPose().theta;
            }
            if (fabsf(_turnAccum) < TURN_ANGLE) {
                return AC{ -TURN_SPEED, TURN_SPEED };  // pivot sur place vers la gauche
            }
            _turning        = false;
            _turnAngleSaved = false;
        }

        // ── Détection coin — les deux capteurs avant sur le blanc ────
        SensorData lineFL = ctx.getLineData(SensorPosition::FRONT_LEFT);
        SensorData lineFR = ctx.getLineData(SensorPosition::FRONT_RIGHT);

        const bool onWhiteL = lineFL.isValid && lineFL.value.scalar > 0.0f;
        const bool onWhiteR = lineFR.isValid && lineFR.value.scalar > 0.0f;

        if (onWhiteL && onWhiteR) {
            _turning = true;
            return AC{ -TURN_SPEED, TURN_SPEED };  // premier cycle du virage
        }

        // ── Suivi de bord — corrige en continu avec le capteur droit ──
        // Sur le blanc → trop à droite, vire à gauche pour revenir sur le noir.
        // Sur le noir → trop à gauche, vire à droite pour retrouver la ligne.
        return onWhiteR ? AC{ FWD_SPEED - TURN_DIFF, FWD_SPEED + TURN_DIFF }
                         : AC{ FWD_SPEED + TURN_DIFF, FWD_SPEED - TURN_DIFF };
    }

private:
    bool  _turning        = false;
    bool  _turnAngleSaved = false;
    float _turnPrevTheta  = 0.0f;
    float _turnAccum      = 0.0f;  // rotation cumulée du virage en cours (rad)

    static constexpr int   FWD_SPEED  = 150;
    static constexpr int   TURN_DIFF  = 40;   // différentiel PWM du suivi de bord — augmenter si le robot décroche la ligne
    static constexpr int   TURN_SPEED = 150;
    static constexpr float TURN_ANGLE = (float)M_PI / 2.0f;  // 90°
};
