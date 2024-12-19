import 'package:flutter/material.dart';

class KeyboardProvider extends ChangeNotifier {
  TextEditingController? _textController;
  FocusNode? _focusNode;
  bool isMaskingUnfocus = false;

  TextEditingController? get textController => _textController;
  set textController(TextEditingController? value) {
    if (isMaskingUnfocus) return;
    _textController = value;
    if (textController != null) {
      focusNode?.requestFocus();
    } else {
      focusNode == null;
    }
    notifyListeners();
  }

  FocusNode? get focusNode => _focusNode;
  set focusNode(FocusNode? value) {
    if (isMaskingUnfocus) return;
    _focusNode = value;
  }
}
