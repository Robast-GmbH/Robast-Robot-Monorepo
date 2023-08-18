import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';

class RoundedButton extends StatelessWidget {
  const RoundedButton({
    super.key,
    required this.text,
    required this.color,
    this.textColor = AppColors.white,
    this.onTap,
    required this.borderRadius,
  });
  final String text;
  final Color color;
  final Color textColor;
  final VoidCallback? onTap;
  final BorderRadius borderRadius;

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.all(16),
        decoration: BoxDecoration(borderRadius: borderRadius, color: color),
        child: Text(
          text,
          style: TextStyle(color: textColor, fontSize: 18, fontWeight: FontWeight.w300),
          textAlign: TextAlign.center,
        ),
      ),
    );
  }
}
