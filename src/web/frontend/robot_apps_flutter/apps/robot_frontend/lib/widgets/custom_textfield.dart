import 'dart:async';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/custom_focus_node.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';

class CustomTextfield extends StatefulWidget {
  const CustomTextfield({this.focusNode, this.onChanged, this.enabledAutofocus = false, this.mainAxisAlignment = MainAxisAlignment.start, super.key});

  final CustomFocusNode? focusNode;
  final bool enabledAutofocus;
  final void Function(String)? onChanged;
  final MainAxisAlignment mainAxisAlignment;

  @override
  State<CustomTextfield> createState() => _CustomTextfieldState();
}

class _CustomTextfieldState extends State<CustomTextfield> {
  late final CustomFocusNode _focusNode;
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
      Provider.of<KeyboardProvider>(context, listen: false).focusNode = _focusNode;
    } else {
      Provider.of<KeyboardProvider>(context, listen: false).focusSilently(_focusNode);
    }
  }

  @override
  void initState() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (widget.enabledAutofocus) {
        focus();
      }
    });
    _focusNode = widget.focusNode ??
        CustomFocusNode(
          key: GlobalKey(),
          text: '',
        );
    _focusNode.setTextState = () {
      setState(() {});
    };
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
        print(_focusNode.key.toString());
        print("onTap");
        focus();
      },
      child: Container(
        color: Colors.transparent,
        child: Selector<KeyboardProvider, CustomFocusNode?>(
          selector: (context, provider) => provider.focusNode,
          builder: (context, focusNode, child) {
            final isFocused = focusNode?.key == _focusNode.key;
            if (isFocused && !(indicatorBlinkingTimer?.isActive ?? false)) {
              indicatorBlinkingTimer = createIndicatorBlinkingTimer();
            } else if (!isFocused) {
              indicatorBlinkingTimer?.cancel();
              indicatorBlinkingValue = true;
            }
            widget.onChanged?.call(_focusNode.text);
            return Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Padding(
                  padding: const EdgeInsets.symmetric(vertical: 4, horizontal: 4),
                  child: Row(
                    mainAxisAlignment: widget.mainAxisAlignment,
                    children: [
                      Text(
                        _focusNode.text,
                        maxLines: 1,
                        style: const TextStyle(fontSize: 32, color: RobotColors.primaryText),
                        overflow: TextOverflow.ellipsis,
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
