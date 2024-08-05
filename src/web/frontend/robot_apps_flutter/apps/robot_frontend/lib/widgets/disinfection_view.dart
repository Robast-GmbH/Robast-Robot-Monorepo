import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';

class DisinfectionView extends StatefulWidget {
  const DisinfectionView({required this.onDisinfection, super.key});

  final VoidCallback onDisinfection;

  @override
  State<DisinfectionView> createState() => _DisinfectionViewState();
}

class _DisinfectionViewState extends State<DisinfectionView> {
  late Future<bool> disinfectionFuture;

  @override
  void initState() {
    super.initState();
    disinfectionFuture = Provider.of<RobotProvider>(context, listen: false).waitForDisinfectionTriggered(
      timeout: 20,
      onDisinfection: widget.onDisinfection,
    );
  }

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: widget.onDisinfection,
      child: const ColoredBox(
        color: Colors.transparent,
        child: Center(
          child: Text(
            'Bitte Hände desinfizieren',
            style: TextStyle(fontSize: 80),
          ),
        ),
      ),
    );
  }
}
