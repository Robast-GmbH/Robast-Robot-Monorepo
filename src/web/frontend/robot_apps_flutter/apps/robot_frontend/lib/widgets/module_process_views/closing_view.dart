import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/centered_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';

class ClosingView extends StatelessWidget {
  const ClosingView({required this.position, super.key});
  final int position;

  @override
  Widget build(BuildContext context) {
    return CenteredView(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 64),
        child: HintView(
          text: 'Schublade schließt sich, bitte warten',
          moduleLabel: 'Modul $position',
        ),
      ),
    );
  }
}
