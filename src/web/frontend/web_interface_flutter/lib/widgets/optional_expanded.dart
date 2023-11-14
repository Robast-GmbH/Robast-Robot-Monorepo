import 'package:flutter/material.dart';

class OptionalExpanded extends StatelessWidget {
  const OptionalExpanded({
    super.key,
    required this.shouldExpand,
    required this.child,
  });

  final bool shouldExpand;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return shouldExpand ? Expanded(child: child) : child;
  }
}
