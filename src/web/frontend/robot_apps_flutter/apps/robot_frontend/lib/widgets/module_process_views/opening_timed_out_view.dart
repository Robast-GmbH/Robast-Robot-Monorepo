import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:shared_data_models/shared_data_models.dart';

class OpeningTimedOutView extends StatelessWidget {
  const OpeningTimedOutView({required this.submodule, super.key});
  final Submodule submodule;
  @override
  Widget build(BuildContext context) {
    final moduleProvider = Provider.of<ModuleProvider>(context, listen: false);
    return Row(
      children: [
        const Expanded(child: SizedBox()),
        Expanded(
          child: Padding(
            padding: const EdgeInsets.symmetric(vertical: 192),
            child: Column(
              children: [
                Expanded(
                  flex: 2,
                  child: CustomButtonView(
                    text: 'Erneut öffnen',
                    titleFontSize: 56,
                    onPressed: () async {
                      await moduleProvider.openSubmodule(submodule);
                    },
                  ),
                ),
                const SizedBox(
                  height: 8,
                ),
                Expanded(
                  child: CustomButtonView(
                    text: 'Abbrechen',
                    titleFontSize: 56,
                    onPressed: () async {
                      await moduleProvider.cancelSubmoduleProcess(submodule);
                      if (context.mounted) {
                        Navigator.pop(context);
                      }
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
        const Expanded(child: SizedBox()),
      ],
    );
  }
}
