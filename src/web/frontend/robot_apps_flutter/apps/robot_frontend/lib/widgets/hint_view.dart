import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class HintView extends StatelessWidget {
  const HintView({
    required this.moduleLabel,
    required this.text,
    super.key,
  });

  final String moduleLabel;
  final String text;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.lightBlue,
        borderRadius: BorderRadius.circular(12),
      ),
      margin: const EdgeInsets.all(4),
      child: SizedBox.expand(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Padding(
              padding: const EdgeInsets.only(top: 8),
              child: Text(
                moduleLabel,
                style: const TextStyle(
                  height: 0,
                  color: RobotColors.primaryText,
                  fontSize: 40,
                  fontWeight: FontWeight.w400,
                ),
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 48),
              child: Text(
                text,
                textAlign: TextAlign.center,
                style: const TextStyle(
                  color: RobotColors.primaryText,
                  fontSize: 40,
                  fontWeight: FontWeight.w500,
                ),
              ),
            ),
            const Padding(
              padding: EdgeInsets.only(bottom: 64),
              child: Icon(
                Icons.arrow_downward,
                size: 100,
                color: RobotColors.primaryText,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
