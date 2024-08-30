import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/data/svgs.dart';

class DrivingView extends StatefulWidget {
  const DrivingView({required this.onPressed, super.key});

  final Future<void> Function() onPressed;

  @override
  State<DrivingView> createState() => _DrivingViewState();
}

class _DrivingViewState extends State<DrivingView> {
  bool isPressed = false;

  bool isEyeClosed = false;

  int blinkSteps = 0;

  Timer? animationTimer;

  void blinking() {
    if (blinkSteps == 0) {
      animationTimer = Timer(const Duration(milliseconds: 6000), () {
        isEyeClosed = true;
        if (mounted) setState(() {});
        blinkSteps = 1;
        blinking();
      });
    } else if (blinkSteps == 1) {
      animationTimer = Timer(const Duration(milliseconds: 100), () {
        isEyeClosed = false;
        if (mounted) setState(() {});
        blinkSteps = 0;
        blinking();
      });
    }
  }

  @override
  void initState() {
    super.initState();
    blinking();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: GestureDetector(
            onTapDown: (details) {
              isPressed = true;
              setState(() {});
            },
            onTapCancel: () {
              isPressed = false;
              setState(() {});
            },
            onTapUp: (_) async {
              await widget.onPressed();
              isPressed = false;
              setState(() {});
            },
            child: Container(
              color: Colors.transparent,
              margin: const EdgeInsets.all(16),
              padding: const EdgeInsets.all(180) - const EdgeInsets.only(top: 32),
              height: double.infinity,
              child: Center(
                child: Column(
                  children: [
                    Expanded(
                      child: Stack(
                        children: [
                          Stack(
                            children: [
                              if (!isPressed)
                                Stack(
                                  children: [
                                    if (!isEyeClosed)
                                      SvgPicture.string(
                                        logo,
                                        width: double.infinity,
                                        color: RobotColors.primaryIcon,
                                      ),
                                    SvgPicture.string(
                                      logoBlink,
                                      width: double.infinity,
                                      color: RobotColors.primaryIcon,
                                    ),
                                  ],
                                )
                              else
                                SvgPicture.string(
                                  logoHappy,
                                  width: double.infinity,
                                  color: RobotColors.primaryIcon,
                                ),
                            ],
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(height: 64),
                    Container(
                      decoration: BoxDecoration(borderRadius: BorderRadius.circular(64), color: RobotColors.primaryText),
                      child: const Padding(
                        padding: EdgeInsets.symmetric(vertical: 8, horizontal: 92),
                        child: Text(
                          'Bitte berühren',
                          style: TextStyle(color: RobotColors.primaryBackground, fontSize: 66),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ],
    );
  }
}
