import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class ContentDistributionTaskCreationPage extends StatefulWidget {
  const ContentDistributionTaskCreationPage({super.key});

  @override
  State<ContentDistributionTaskCreationPage> createState() => _ContentDistributionTaskCreationPageState();
}

class _ContentDistributionTaskCreationPageState extends State<ContentDistributionTaskCreationPage> {
  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      title: 'Inhalte verteilen',
    );
  }
}
