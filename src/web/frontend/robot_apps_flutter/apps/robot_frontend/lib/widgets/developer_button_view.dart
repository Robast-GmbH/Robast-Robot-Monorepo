import 'dart:async';

import 'package:flutter/material.dart';

class DeveloperButtonView extends StatefulWidget {
  const DeveloperButtonView({
    super.key,
    required this.child,
    required this.onPressed,
  });

  final Widget child;
  final VoidCallback onPressed;

  @override
  State<DeveloperButtonView> createState() => _DeveloperButtonViewState();
}

class _DeveloperButtonViewState extends State<DeveloperButtonView> {
  int _tapCount = 0;

  Timer? _tapTimer;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        if (_tapTimer != null) {
          _tapTimer!.cancel();
        }
        _tapCount++;
        if (_tapCount >= 7) {
          widget.onPressed();
          _tapCount = 0;
        }
        _tapTimer = Timer(Duration(seconds: 2), () {
          _tapCount = 0;
        });
      },
      child: widget.child,
    );
  }
}
