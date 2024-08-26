import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';

class CustomButtonView extends StatelessWidget {
  const CustomButtonView({
    required this.onPressed,
    this.content = const SizedBox(),
    this.padding = EdgeInsets.zero,
    this.trailing = const SizedBox(),
    this.text,
    this.header,
    super.key,
  });
  final String? text;
  final Widget content;
  final Widget trailing;
  final VoidCallback onPressed;
  final EdgeInsets padding;
  final Widget? header;
  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onPressed,
      child: Container(
        decoration: BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.bottomCenter,
            end: Alignment.topCenter,
            colors: [
              Colors.black.withOpacity(0.2),
              Colors.black.withOpacity(0.2),
            ],
          ),
          borderRadius: BorderRadius.circular(16),
        ),
        child: Padding(
          padding: padding,
          child: Stack(
            fit: StackFit.expand,
            children: [
              Align(
                alignment: Alignment.topLeft,
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Stack(
                    children: [
                      Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          if (header != null) ...[
                            header!
                          ] else ...[
                            Text(
                              text ?? '',
                              textAlign: TextAlign.center,
                              style: const TextStyle(
                                height: 0,
                                color: RobotColors.primaryText,
                                fontSize: 40,
                                fontWeight: FontWeight.w400,
                              ),
                            ),
                          ],
                          const SizedBox(height: 4),
                          Expanded(child: content),
                        ],
                      ),
                      Align(
                        alignment: Alignment.bottomRight,
                        child: trailing,
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
