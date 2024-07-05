// moduleProcessStatus: json['module_process_status'] as String,
// moduleProcessType: json['module_process_type'] as String,
// moduleProcessPayload: Map<String, int>.from(json['module_process_payload'] as Map<String, dynamic>),
enum ModuleProcessStatus { idle, auth, waitingForOpening, opening, open, closing, closed }

class ModuleProcess {
  ModuleProcess({
    required this.status,
    required this.type,
    required this.payload,
  });

  factory ModuleProcess.fromJson(Map<String, dynamic> json) {
    return ModuleProcess(
      status: moduleProcessStatusFromString(json['module_process_status'] as String),
      type: json['module_process_type'] as String,
      payload: Map<String, int>.from(json['module_process_payload'] as Map<String, dynamic>),
    );
  }

  final ModuleProcessStatus status;
  final String type;
  final Map<String, int> payload;

  String payloadToString() {
    final pickupObjects = payload.entries.where((entry) => entry.value > 0).toList();
    final dropoffObjects = payload.entries.where((entry) => entry.value < 0).toList();

    var pickupString = '';
    var dropoffString = '';

    if (pickupObjects.isNotEmpty) {
      pickupString = "${pickupObjects.map((entry) => '${entry.value} ${entry.key}').join(', ')} in die Schublade legen";
    }

    if (dropoffObjects.isNotEmpty) {
      dropoffString = "${dropoffObjects.map((entry) => '${-entry.value} ${entry.key}').join(', ')} aus der Schublade nehmen";
    }

    if (pickupString.isNotEmpty && dropoffString.isNotEmpty) {
      return 'Bitte $pickupString und $dropoffString.';
    } else if (pickupString.isNotEmpty) {
      return 'Bitte $pickupString.';
    } else if (dropoffString.isNotEmpty) {
      return 'Bitte $dropoffString.';
    } else {
      return 'Es gibt nichts zu tun.';
    }
  }

  static ModuleProcessStatus moduleProcessStatusFromString(String data) {
    switch (data) {
      case 'idle':
        return ModuleProcessStatus.idle;
      case 'auth':
        return ModuleProcessStatus.auth;
      case 'waiting_for_opening':
        return ModuleProcessStatus.waitingForOpening;
      case 'opening':
        return ModuleProcessStatus.opening;
      case 'open':
        return ModuleProcessStatus.open;
      case 'closing':
        return ModuleProcessStatus.closing;
      case 'closed':
        return ModuleProcessStatus.closed;
      default:
        throw ArgumentError('Invalid enum value for ModuleProcessState');
    }
  }
}
