import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/centered_view.dart';
import 'package:robot_frontend/widgets/hint_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class OpenView extends StatelessWidget {
  const OpenView({required this.submodule, required this.position, super.key});
  final Submodule submodule;
  final int position;
  @override
  Widget build(BuildContext context) {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    return CenteredView(
      child: Padding(
        padding: const EdgeInsets.symmetric(vertical: 64),
        child: Stack(
          children: [
            if (submodule.moduleProcess.status == ModuleProcessStatus.open) ...[
              GestureDetector(
                onTap: () {
                  if (submodule.variant == SubmoduleVariant.electric || submodule.variant == SubmoduleVariant.partial) {
                    moduleProvider.closeSubmodule(submodule);
                  }
                },
                child: HintView(
                  text:
                      '${submodule.moduleProcess.itemsByChangeToString()}${submodule.variant == SubmoduleVariant.electric || submodule.variant == SubmoduleVariant.partial ? ' Zum Schließen tippen.' : ''}',
                  moduleLabel: 'Modul $position',
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }
}
