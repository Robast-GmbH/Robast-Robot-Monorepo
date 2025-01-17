import 'package:flutter/material.dart';
import 'package:virtual_keyboard_custom_layout/virtual_keyboard_custom_layout.dart';

class CustomFocusNode {
  CustomFocusNode({
    required this.key,
    required this.text,
    this.setTextState,
    this.next,
    this.layout = VirtualKeyboardDefaultLayouts.German,
  });

  GlobalKey key;
  String text;
  VoidCallback? setTextState;
  CustomFocusNode? next;
  VirtualKeyboardDefaultLayouts layout;
}
