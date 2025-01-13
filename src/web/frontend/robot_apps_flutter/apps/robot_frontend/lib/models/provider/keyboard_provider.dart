import 'package:flutter/material.dart';

class KeyboardProvider extends ChangeNotifier {
  String? text;
  VoidCallback? setTextState;
  GlobalKey? _key;
  GlobalKey? get key => _key;
  set key(GlobalKey? value) {
    if (value == _key) return;
    print('setKey');
    _key = value;

    notifyListeners();
  }

  void setKeyWithoutNotify(GlobalKey? value) {
    _key = value;
  }

  void unfocus() {
    text = null;
    setTextState = null;
    _key = null;
    notifyListeners();
  }
}
