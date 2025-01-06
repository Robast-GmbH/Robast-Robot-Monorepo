import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';

class CustomTextfield extends StatefulWidget {
  const CustomTextfield({this.textController, super.key});

  final TextController? textController;

  @override
  State<CustomTextfield> createState() => _CustomTextfieldState();
}

class _CustomTextfieldState extends State<CustomTextfield> {
  final _key = GlobalKey();
  late TextController controller;
  Timer? indicatorBlinkingTimer;
  bool indicatorBlinkingValue = true;

  @override
  void initState() {
    controller = widget.textController ?? TextController();
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        print("onTap");
        Provider.of<KeyboardProvider>(context, listen: false).key = _key;
        Provider.of<KeyboardProvider>(context, listen: false).text = controller.text;
        Provider.of<KeyboardProvider>(context, listen: false).setTextState = () {
          setState(() {
            controller.text = Provider.of<KeyboardProvider>(context, listen: false).text ?? '';
          });
        };
      },
      child: Container(
        color: Colors.transparent,
        child: Selector<KeyboardProvider, GlobalKey?>(
          selector: (context, provider) => provider.key,
          builder: (context, key, child) {
            final isFocused = key == _key;
            if (isFocused && !(indicatorBlinkingTimer?.isActive ?? false)) {
              indicatorBlinkingTimer = Timer.periodic(const Duration(milliseconds: 500), (timer) {
                setState(() {
                  indicatorBlinkingValue = !indicatorBlinkingValue;
                });
              });
            } else if (!isFocused) {
              indicatorBlinkingTimer?.cancel();
              indicatorBlinkingValue = true;
            }
            return Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 8.0),
                  child: Row(
                    children: [
                      Text(
                        controller.text,
                        maxLines: 1,
                        style: const TextStyle(fontSize: 18),
                      ),
                      if (isFocused)
                        Container(
                          margin: const EdgeInsets.only(left: 2),
                          height: 18,
                          width: 1,
                          color: indicatorBlinkingValue ? Colors.black : Colors.transparent,
                        )
                    ],
                  ),
                ),
                Container(
                  color: isFocused ? Colors.blue : Colors.grey,
                  width: double.infinity,
                  height: 1,
                ),
              ],
            );
          },
        ),
      ),
    );
  }
}
