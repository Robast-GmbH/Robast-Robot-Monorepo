class TextController {
  TextController({this.text = ''});

  String text = '';
  bool isFocused = false;

  void clear() {
    text = '';
  }
}
