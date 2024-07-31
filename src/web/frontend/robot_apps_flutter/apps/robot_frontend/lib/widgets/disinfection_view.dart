import 'package:flutter/material.dart';

class DisinfectionView extends StatelessWidget {
  const DisinfectionView({required this.onDisinfection, super.key});

  final VoidCallback onDisinfection;

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onDisinfection,
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
