import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/centered_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class OpeningView extends StatelessWidget {
  const OpeningView({required this.submodule, required this.position, super.key});
  final Submodule submodule;
  final int position;

  @override
  Widget build(BuildContext context) {
    return CenteredView(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 64),
        child: HintView(
          text: submodule.variant == SubmoduleVariant.electric || submodule.variant == SubmoduleVariant.partial
              ? 'Gewählte Schublade öffnet sich'
              : 'Bitte gewählte Schublade öffnen',
          moduleLabel: 'Modul $position',
        ),
      ),
    );
  }
}
