import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class PickupTaskCreationPage extends StatefulWidget {
  const PickupTaskCreationPage({super.key});

  @override
  State<PickupTaskCreationPage> createState() => _PickupTaskCreationPageState();
}

class _PickupTaskCreationPageState extends State<PickupTaskCreationPage> {
  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      title: 'Abholung',
      child: Padding(
        padding: EdgeInsets.all(128),
        child: Column(),
      ),
    );
  }
}
