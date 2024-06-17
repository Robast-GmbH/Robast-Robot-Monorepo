enum ModuleProcessState { waitingForOpeningCommand, opening, open, closing, closed, finished }

extension ModuleProcessStateExtension on ModuleProcessState {}

class ModuleProcess {
  final int moduleID;
  final int drawerID;
  final String processName;
  final String payload;
  final ModuleProcessState state;

  const ModuleProcess({
    required this.moduleID,
    required this.drawerID,
    required this.processName,
    required this.payload,
    required this.state,
  });

  static ModuleProcessState moduleProcessStateFromString(String data) {
    switch (data) {
      case 'waiting_for_opening_command':
        return ModuleProcessState.waitingForOpeningCommand;
      case 'opening':
        return ModuleProcessState.opening;
      case 'open':
        return ModuleProcessState.open;
      case 'closing':
        return ModuleProcessState.closing;
      case 'closed':
        return ModuleProcessState.closed;
      case 'finished':
        return ModuleProcessState.finished;
      default:
        throw ArgumentError('Invalid enum value for ModuleProcessState');
    }
  }

  static fromJson(Map<String, dynamic> json) => ModuleProcess(
        moduleID: json['module_id'] as int,
        drawerID: json['drawer_id'] as int,
        processName: json['process_name'] as String,
        payload: json['payload'] as String,
        state: moduleProcessStateFromString(json['state'] as String),
      );
}
