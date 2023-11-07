import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';

class RobotMarker extends StatelessWidget {
  const RobotMarker({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        borderRadius: BorderRadius.all(Radius.circular(4)),
        color: AppColors.blue,
      ),
      child: const Icon(
        Icons.arrow_forward,
        size: 24,
      ),
    );
  }
}
