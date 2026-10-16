using System.Windows.Input;
using Avalonia.Threading;
using RobotHMI.Services;

namespace RobotHMI.ViewModels;

// ---------------------------------------------------------------------------
// TuningPanelViewModel — MODIFIÉ V2.3
// ---------------------------------------------------------------------------
// Changements V2.3 :
//   - Ajout des commandes PID YAW (KP, KI, KD) via SerializePidYaw()
//   - Valeurs initiales YAW : KP=0.5, KI=0.0, KD=0.0 (firmware)
// ---------------------------------------------------------------------------

public class TuningPanelViewModel : ViewModelBase
{
    private readonly RobotCommunicationService _commService;

    // ── Valeurs initiales PID moteurs — synchronisées avec main.cpp ─────────
    // PID pidLeft(2.0f, 0.15f, 0.0f) / pidRight(2.0f, 0.15f, 0.0f)
    private string _kpText = "2.0";
    private string _kiText = "0.15";
    private string _kdText = "0.0";

    // ── Valeurs initiales PID moteur gauche/droit seul — synchronisées avec main.cpp ──
    // PID g_pidLeftKp/Ki/Kd et g_pidRightKp/Ki/Kd (identiques au départ)
    private string _leftKpText = "2.0";
    private string _leftKiText = "0.15";
    private string _leftKdText = "0.0";
    private string _rightKpText = "2.0";
    private string _rightKiText = "0.15";
    private string _rightKdText = "0.0";

    // ── Rotation d'une roue seule (test/calibration) ─────────────────────
    private string _leftWheelDegText  = "180";
    private string _rightWheelDegText = "180";

    // ── Valeurs initiales PID YAW — synchronisées avec main.cpp ──────────
    // PID pidYaw(5.0f, 0.5f, 0.0f)
    private string _yawKpText = "5.0";
    private string _yawKiText = "0.5";
    private string _yawKdText = "0.0";

    public TuningPanelViewModel(RobotCommunicationService commService)
    {
        _commService = commService;

        // PID moteurs
        SendKpCommand = new AsyncRelayCommand(OnSendKpAsync);
        SendKiCommand = new AsyncRelayCommand(OnSendKiAsync);
        SendKdCommand = new AsyncRelayCommand(OnSendKdAsync);

        // PID moteur gauche/droit seul
        SendLeftKpCommand  = new AsyncRelayCommand(OnSendLeftKpAsync);
        SendLeftKiCommand  = new AsyncRelayCommand(OnSendLeftKiAsync);
        SendLeftKdCommand  = new AsyncRelayCommand(OnSendLeftKdAsync);
        SendRightKpCommand = new AsyncRelayCommand(OnSendRightKpAsync);
        SendRightKiCommand = new AsyncRelayCommand(OnSendRightKiAsync);
        SendRightKdCommand = new AsyncRelayCommand(OnSendRightKdAsync);

        // PID YAW — MODIFIÉ V2.3
        SendYawKpCommand = new AsyncRelayCommand(OnSendYawKpAsync);
        SendYawKiCommand = new AsyncRelayCommand(OnSendYawKiAsync);
        SendYawKdCommand = new AsyncRelayCommand(OnSendYawKdAsync);

        // Rotation roue seule
        SendLeftWheelCommand  = new AsyncRelayCommand(OnSendLeftWheelAsync);
        SendRightWheelCommand = new AsyncRelayCommand(OnSendRightWheelAsync);
    }

    // ── Propriétés PID moteurs ────────────────────────────────────────────

    public string KpText { get => _kpText; set => SetField(ref _kpText, value); }
    public string KiText { get => _kiText; set => SetField(ref _kiText, value); }
    public string KdText { get => _kdText; set => SetField(ref _kdText, value); }

    // ── Propriétés PID moteur gauche/droit seul ──────────────────────────

    public string LeftKpText  { get => _leftKpText;  set => SetField(ref _leftKpText,  value); }
    public string LeftKiText  { get => _leftKiText;  set => SetField(ref _leftKiText,  value); }
    public string LeftKdText  { get => _leftKdText;  set => SetField(ref _leftKdText,  value); }
    public string RightKpText { get => _rightKpText; set => SetField(ref _rightKpText, value); }
    public string RightKiText { get => _rightKiText; set => SetField(ref _rightKiText, value); }
    public string RightKdText { get => _rightKdText; set => SetField(ref _rightKdText, value); }

    // ── Propriétés rotation roue seule ────────────────────────────────────

    public string LeftWheelDegText  { get => _leftWheelDegText;  set => SetField(ref _leftWheelDegText,  value); }
    public string RightWheelDegText { get => _rightWheelDegText; set => SetField(ref _rightWheelDegText, value); }

    // ── Propriétés PID YAW — MODIFIÉ V2.3 ────────────────────────────────

    public string YawKpText { get => _yawKpText; set => SetField(ref _yawKpText, value); }
    public string YawKiText { get => _yawKiText; set => SetField(ref _yawKiText, value); }
    public string YawKdText { get => _yawKdText; set => SetField(ref _yawKdText, value); }

    // ── Commandes PID moteurs ─────────────────────────────────────────────

    public ICommand SendKpCommand { get; }
    public ICommand SendKiCommand { get; }
    public ICommand SendKdCommand { get; }

    private async Task OnSendKpAsync() => await SendPid("KP", KpText);
    private async Task OnSendKiAsync() => await SendPid("KI", KiText);
    private async Task OnSendKdAsync() => await SendPid("KD", KdText);

    // ── Commandes PID moteur gauche/droit seul ───────────────────────────

    public ICommand SendLeftKpCommand  { get; }
    public ICommand SendLeftKiCommand  { get; }
    public ICommand SendLeftKdCommand  { get; }
    public ICommand SendRightKpCommand { get; }
    public ICommand SendRightKiCommand { get; }
    public ICommand SendRightKdCommand { get; }

    private async Task OnSendLeftKpAsync()  => await SendPidLeft("KP", LeftKpText);
    private async Task OnSendLeftKiAsync()  => await SendPidLeft("KI", LeftKiText);
    private async Task OnSendLeftKdAsync()  => await SendPidLeft("KD", LeftKdText);
    private async Task OnSendRightKpAsync() => await SendPidRight("KP", RightKpText);
    private async Task OnSendRightKiAsync() => await SendPidRight("KI", RightKiText);
    private async Task OnSendRightKdAsync() => await SendPidRight("KD", RightKdText);

    // ── Commandes rotation roue seule ─────────────────────────────────────

    public ICommand SendLeftWheelCommand  { get; }
    public ICommand SendRightWheelCommand { get; }

    private async Task OnSendLeftWheelAsync()
    {
        if (!TryParseFloat(LeftWheelDegText, out float deg, "WHEEL:LEFT")) return;
        await SendSafe(CommandSerializer.SerializeWheelLeft(deg));
    }

    private async Task OnSendRightWheelAsync()
    {
        if (!TryParseFloat(RightWheelDegText, out float deg, "WHEEL:RIGHT")) return;
        await SendSafe(CommandSerializer.SerializeWheelRight(deg));
    }

    // ── Commandes PID YAW — MODIFIÉ V2.3 ─────────────────────────────────

    public ICommand SendYawKpCommand { get; }
    public ICommand SendYawKiCommand { get; }
    public ICommand SendYawKdCommand { get; }

    private async Task OnSendYawKpAsync() => await SendPidYaw("KP", YawKpText);
    private async Task OnSendYawKiAsync() => await SendPidYaw("KI", YawKiText);
    private async Task OnSendYawKdAsync() => await SendPidYaw("KD", YawKdText);

    // ── Helpers d'envoi ───────────────────────────────────────────────────

    private async Task SendPid(string parameter, string valueText)
    {
        if (!TryParseFloat(valueText, out float value, parameter)) return;
        await SendSafe(CommandSerializer.SerializePid(parameter, value));
    }

    private async Task SendPidYaw(string parameter, string valueText)  // MODIFIÉ V2.3
    {
        if (!TryParseFloat(valueText, out float value, $"YAW:{parameter}")) return;
        await SendSafe(CommandSerializer.SerializePidYaw(parameter, value));
    }

    private async Task SendPidLeft(string parameter, string valueText)
    {
        if (!TryParseFloat(valueText, out float value, $"LEFT:{parameter}")) return;
        await SendSafe(CommandSerializer.SerializePidLeft(parameter, value));
    }

    private async Task SendPidRight(string parameter, string valueText)
    {
        if (!TryParseFloat(valueText, out float value, $"RIGHT:{parameter}")) return;
        await SendSafe(CommandSerializer.SerializePidRight(parameter, value));
    }

    private async Task SendSafe(string rawCommand)
    {
        try
        {
            await _commService.SendAsync(rawCommand);
            await Dispatcher.UIThread.InvokeAsync(() => StatusText = $"Envoyé : {rawCommand}");
        }
        catch (Exception ex)
        {
            await Dispatcher.UIThread.InvokeAsync(() => StatusText = $"Erreur : {ex.Message}");
        }
    }

    private bool TryParseFloat(string text, out float value, string label)
    {
        if (float.TryParse(text,
                System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture,
                out value))
            return true;

        Dispatcher.UIThread.InvokeAsync(() =>
            StatusText = $"Valeur invalide pour {label} : '{text}'");
        value = 0f;
        return false;
    }

    // ── Feedback ─────────────────────────────────────────────────────────

    private string _statusText = "Aucune commande envoyée";
    public string StatusText
    {
        get => _statusText;
        private set => SetField(ref _statusText, value);
    }
}
