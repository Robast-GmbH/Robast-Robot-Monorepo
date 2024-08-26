enum SubmoduleTypes { small, medium, large, partial }

extension SubmoduleTypesExtension on SubmoduleTypes {
  String get value {
    switch (this) {
      case SubmoduleTypes.small:
        return 'small';
      case SubmoduleTypes.medium:
        return 'medium';
      case SubmoduleTypes.large:
        return 'large';
      case SubmoduleTypes.partial:
        return 'partial';
    }
  }
}
