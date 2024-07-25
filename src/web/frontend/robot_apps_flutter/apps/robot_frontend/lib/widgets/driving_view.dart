import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:robot_frontend/data/svgs.dart';

class DrivingView extends StatefulWidget {
  DrivingView({super.key, required this.onPressed});

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
                                        color: Colors.white,
                                      ),
                                    SvgPicture.string(
                                      logoBlink,
                                      width: double.infinity,
                                      color: Colors.white,
                                    ),
                                  ],
                                )
                              else
                                SvgPicture.string(
                                  logoHappy,
                                  width: double.infinity,
                                  color: Colors.white,
                                )
                            ],
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(height: 64),
                    Stack(
                      alignment: Alignment.center,
                      children: [
                        ShaderMask(
                          blendMode: BlendMode.srcOut,
                          child: Text(
                            'Bitte berühren',
                            style: TextStyle(
                              letterSpacing: 1.5,
                              color: Color(0xFF755FE3),
                              fontSize: 72,
                              fontWeight: FontWeight.w300,
                            ),
                          ),
                          shaderCallback: (bounds) => LinearGradient(colors: [Colors.white], stops: [0.0]).createShader(bounds),
                        ),
                        Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Container(
                              decoration: BoxDecoration(
                                color: Colors.white,
                                borderRadius: BorderRadius.horizontal(left: Radius.circular(64)),
                              ),
                              width: 96,
                              height: 85,
                            ),
                            Column(
                              mainAxisSize: MainAxisSize.min,
                              mainAxisAlignment: MainAxisAlignment.start,
                              children: [
                                Text(
                                  'Bitteberühren',
                                  style: TextStyle(
                                    letterSpacing: 1.9,
                                    color: Colors.transparent,
                                    fontSize: 58,
                                    fontWeight: FontWeight.w300,
                                  ),
                                ),
                                Container(
                                  color: Colors.white,
                                  width: 480,
                                  height: 2,
                                )
                              ],
                            ),
                            Container(
                              decoration: BoxDecoration(
                                color: Colors.white,
                                borderRadius: BorderRadius.horizontal(right: Radius.circular(64)),
                              ),
                              width: 96,
                              height: 85,
                            ),
                          ],
                        )
                      ],
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
