import 'package:flutter/material.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class RoundedButton extends StatelessWidget {
  const RoundedButton({
    required this.onPressed,
    required this.child,
    super.key,
    this.color,
    this.padding = EdgeInsets.zero,
  });
  final Color? color;
  final Widget child;
  final VoidCallback onPressed;
  final EdgeInsets padding;
  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onPressed,
      child: Padding(
        padding: padding,
        child: RoundedContainer(
          color: color,
          child: Center(child: child),
        ),
      ),
    );
  }
}
