import 'package:flutter/material.dart';

class CenteredView extends StatelessWidget {
  const CenteredView({required this.child, super.key});
  final Widget child;
  @override
  Widget build(BuildContext context) {
    return Center(
      child: FractionallySizedBox(
        widthFactor: 1 / 3,
        child: child,
      ),
    );
  }
}
