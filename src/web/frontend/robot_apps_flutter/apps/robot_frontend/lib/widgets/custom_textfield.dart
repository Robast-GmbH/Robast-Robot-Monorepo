import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';

class CustomTextfield extends StatefulWidget {
  const CustomTextfield({super.key, required this.controller, this.focusNode, this.onChanged});

  final TextEditingController controller;
  final FocusNode? focusNode;
  final void Function(String)? onChanged;

  @override
  State<CustomTextfield> createState() => _CustomTextfieldState();
}

class _CustomTextfieldState extends State<CustomTextfield> {
  late final FocusNode focusNode;
  Timer? maskUnfocusTimer;

  void textEditingControllerCallback() {
    print("Text: ${widget.controller.text}");
    widget.onChanged?.call(widget.controller.text);
    focusNode.requestFocus();
  }

  void focusNodeCallback() {
    if (focusNode.hasFocus) {
      maskUnfocusTimer?.cancel();
      Provider.of<KeyboardProvider>(context, listen: false).focusNode = focusNode;
      Provider.of<KeyboardProvider>(context, listen: false).textController = widget.controller;
    } else {
      print('Unfocus');
      maskUnfocusTimer?.cancel();
      maskUnfocusTimer = Timer(const Duration(milliseconds: 200), () {
        if (mounted && !(Provider.of<KeyboardProvider>(context, listen: false).focusNode?.hasFocus ?? true)) {
          Provider.of<KeyboardProvider>(context, listen: false).focusNode = null;
          Provider.of<KeyboardProvider>(context, listen: false).textController = null;
        }
      });
    }
    print('Focus: ${focusNode.hasFocus}');
  }

  @override
  void initState() {
    super.initState();

    focusNode = widget.focusNode ?? FocusNode();
    focusNode.addListener(focusNodeCallback);
    widget.controller.addListener(textEditingControllerCallback);
  }

  @override
  void dispose() {
    maskUnfocusTimer?.cancel();
    focusNode.removeListener(focusNodeCallback);
    widget.controller.removeListener(textEditingControllerCallback);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return TextField(
      focusNode: focusNode,
      controller: widget.controller,
      style: const TextStyle(fontSize: 32, color: RobotColors.secondaryText),
      decoration: InputDecoration(
        enabledBorder: Provider.of<KeyboardProvider>(context, listen: true).focusNode == focusNode
            ? const UnderlineInputBorder(
                borderSide: BorderSide(color: Colors.white),
              )
            : null,
        focusedBorder: const UnderlineInputBorder(
          borderSide: BorderSide(color: Colors.white),
        ),
      ),
    );
  }
}
