import 'package:flutter/material.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';

class KeyboardProvider extends ChangeNotifier {
  CustomFocusNode? _focusNode;
  CustomFocusNode? get focusNode => _focusNode;
  set focusNode(CustomFocusNode? focusNode) {
    if (focusNode?.key == _focusNode?.key) return;
    _focusNode = focusNode;
    notifyListeners();
  }

  void focusSilently(CustomFocusNode? focusNode) {
    _focusNode = focusNode;
  }

  void unfocus() {
    _focusNode = null;
    notifyListeners();
  }

  void focusNext() {
    if (_focusNode?.next != null) {
      _focusNode = _focusNode?.next;
      notifyListeners();
    } else if (focusNode != null) {
      _focusNode = null;

      notifyListeners();
    }
  }
}
