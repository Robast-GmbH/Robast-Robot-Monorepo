import 'package:flutter/material.dart';

class RobotMapView extends StatelessWidget {
  const RobotMapView({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      height: double.infinity,
      decoration: BoxDecoration(borderRadius: BorderRadius.circular(16), color: Colors.black.withOpacity(0.2)),
      child: const Center(
        child: Icon(
          Icons.map,
          size: 160,
        ),
      ),
    );
  }
}
