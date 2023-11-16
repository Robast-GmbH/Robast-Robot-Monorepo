import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/constants/constants.dart';

class RoundedButton extends StatelessWidget {
  const RoundedButton({
    super.key,
    required this.text,
    required this.color,
    this.textColor = AppColors.white,
    this.padding = Constants.smallPadding,
    this.onTap,
    this.borderRadius,
  });
  final String text;
  final Color color;
  final Color textColor;
  final VoidCallback? onTap;
  final BorderRadius? borderRadius;
  final EdgeInsets padding;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: padding,
      child: ClipRRect(
        borderRadius: borderRadius ?? BorderRadius.circular(64),
        child: Material(
          color: color,
          child: InkWell(
            onTap: onTap,
            child: Container(
              padding: Constants.largePadding,
              child: Text(
                text,
                style: TextStyle(
                  color: textColor,
                  fontSize: 30,
                  fontWeight: FontWeight.w300,
                ),
                textAlign: TextAlign.center,
              ),
            ),
          ),
        ),
      ),
    );
  }
}
