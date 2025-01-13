import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:shared_data_models/shared_data_models.dart';

class CustomTextfield extends StatefulWidget {
  const CustomTextfield({this.textController, this.onChanged, this.enabledAutofocus = false, super.key});

  final TextController? textController;
  final bool enabledAutofocus;
  final void Function(String)? onChanged;

  @override
  State<CustomTextfield> createState() => _CustomTextfieldState();
}

class _CustomTextfieldState extends State<CustomTextfield> {
  final _key = GlobalKey();
  late TextController controller;
  Timer? indicatorBlinkingTimer;
  bool indicatorBlinkingValue = true;

  Timer createIndicatorBlinkingTimer() {
    return Timer.periodic(const Duration(milliseconds: 500), (timer) {
      setState(() {
        indicatorBlinkingValue = !indicatorBlinkingValue;
      });
    });
  }

  void focus({bool shouldNotify = true}) {
    if (shouldNotify) {
      Provider.of<KeyboardProvider>(context, listen: false).key = _key;
    } else {
      Provider.of<KeyboardProvider>(context, listen: false).setKeyWithoutNotify(_key);
    }
    Provider.of<KeyboardProvider>(context, listen: false).text = controller.text;
    Provider.of<KeyboardProvider>(context, listen: false).setTextState = () {
      setState(() {
        controller.text = Provider.of<KeyboardProvider>(context, listen: false).text ?? '';
      });
    };
  }

  @override
  void initState() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (widget.enabledAutofocus) {
        focus();
      }
    });
    controller = widget.textController ?? TextController();
    super.initState();
  }

  @override
  void dispose() {
    indicatorBlinkingTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        print(_key.toString());
        print("onTap");
        focus();
      },
      child: Container(
        color: Colors.transparent,
        child: Selector<KeyboardProvider, GlobalKey?>(
          selector: (context, provider) => provider.key,
          builder: (context, key, child) {
            final isFocused = key == _key;
            if (isFocused && !(indicatorBlinkingTimer?.isActive ?? false)) {
              indicatorBlinkingTimer = createIndicatorBlinkingTimer();
            } else if (!isFocused) {
              indicatorBlinkingTimer?.cancel();
              indicatorBlinkingValue = true;
            }
            widget.onChanged?.call(controller.text);
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
                        style: const TextStyle(fontSize: 26, color: RobotColors.primaryText),
                      ),
                      if (isFocused)
                        Container(
                          margin: const EdgeInsets.only(left: 2),
                          height: 32,
                          width: 1,
                          color: indicatorBlinkingValue ? RobotColors.primaryText : Colors.transparent,
                        )
                    ],
                  ),
                ),
                Container(
                  color: isFocused ? RobotColors.primaryText : RobotColors.tertiaryText,
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
