import 'package:flutter/material.dart';

class KeyboardProvider extends ChangeNotifier {
  String? text;
  VoidCallback? setTextState;
  GlobalKey? _key;
  GlobalKey? get key => _key;
  set key(GlobalKey? value) {
    print('setKey');
    _key = value;
    notifyListeners();
  }
}
