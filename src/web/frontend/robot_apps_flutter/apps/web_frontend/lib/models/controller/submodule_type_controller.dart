class SubmoduleTypeController {
  String? value;

  int valueAsInt() {
    switch (value) {
      case 'partial':
        return 5;
      case 'small':
        return 1;
      case 'medium':
        return 2;
      case 'large':
        return 3;
      default:
        return 1;
    }
  }
}
