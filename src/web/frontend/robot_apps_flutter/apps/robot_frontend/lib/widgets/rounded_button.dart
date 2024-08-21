import 'package:flutter/material.dart';

class RoundedButton extends StatelessWidget {
  const RoundedButton({
    super.key,
    required this.onPressed,
    required this.color,
    required this.child,
    this.padding = EdgeInsets.zero,
  });
  final Color color;
  final Widget child;
  final VoidCallback onPressed;
  final EdgeInsets padding;
  @override
  Widget build(BuildContext context) {
    return InkWell(
        onTap: onPressed,
        child: Padding(
          padding: padding,
          child: Container(
            decoration: BoxDecoration(borderRadius: BorderRadius.circular(16), color: color),
            child: Center(child: child),
          ),
        ));
  }
}
