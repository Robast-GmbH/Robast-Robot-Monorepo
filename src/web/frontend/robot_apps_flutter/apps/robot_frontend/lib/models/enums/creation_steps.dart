enum CreationSteps {
  reserveSubmodules,
  fillModules,
  assignTargets,
}

extension CreationStepsExtension on CreationSteps {
  String get name {
    switch (this) {
      case CreationSteps.reserveSubmodules:
        return 'Module reservieren';
      case CreationSteps.fillModules:
        return 'Module befüllen';
      case CreationSteps.assignTargets:
        return 'Ziele zuweisen';
      default:
        return '';
    }
  }
}
