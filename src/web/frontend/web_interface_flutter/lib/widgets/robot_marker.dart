import 'package:flutter/material.dart';

class RobotMarker extends StatelessWidget {
  const RobotMarker({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        borderRadius: const BorderRadius.all(Radius.circular(4)),
        color: Theme.of(context).colorScheme.inversePrimary,
      ),
      child: const Padding(
        padding: EdgeInsets.all(0),
        child: Icon(
          Icons.arrow_forward,
          size: 16,
        ),
      ),
    );
  }
}
