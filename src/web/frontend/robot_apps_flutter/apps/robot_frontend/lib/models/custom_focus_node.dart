import 'package:flutter/material.dart';
import 'package:virtual_keyboard_custom_layout/virtual_keyboard_custom_layout.dart';

class CustomFocusNode {
  CustomFocusNode({
    required this.key,
    this.layout = VirtualKeyboardDefaultLayouts.German,
    this.maxTextLength = 50,
    this.text = "",
    this.setTextState,
    this.next,
  });

  final GlobalKey key;
  final VirtualKeyboardDefaultLayouts layout;
  final int maxTextLength;

  String text;
  VoidCallback? setTextState;
  CustomFocusNode? next;
}
