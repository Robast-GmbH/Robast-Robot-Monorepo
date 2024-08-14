enum CreationSteps {
  reserveModules,
  fillModules,
  assignTargets,
}

extension CreationStepsExtension on CreationSteps {
  String get name {
    switch (this) {
      case CreationSteps.reserveModules:
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
