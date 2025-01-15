import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/inactivity_provider.dart';
import 'package:robot_frontend/models/provider/keyboard_provider.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/module_pages/module_process_page.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';
import 'package:robot_frontend/widgets/buttons/rounded_button.dart';
import 'package:shared_data_models/shared_data_models.dart';

class ModuleFillingPage extends StatefulWidget {
  const ModuleFillingPage({required this.submodule, super.key});

  final Submodule submodule;

  @override
  State<ModuleFillingPage> createState() => _ModuleFillingPageState();
}

class _ModuleFillingPageState extends State<ModuleFillingPage> {
  final moduleContentController = ModuleContentController();

  @override
  void initState() {
    super.initState();
    moduleContentController.initialItemsByCount.addAll(widget.submodule.itemsByCount);
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Modul befüllen/entladen',
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 128, vertical: 16),
        child: Column(
          children: [
            Expanded(
              child: ModuleContentCreationView(
                moduleContentController: moduleContentController,
                label: 'Inhalt',
              ),
            ),
            if (Provider.of<KeyboardProvider>(context).focusNode != null)
              const SizedBox(
                height: 280,
              ),
            Padding(
              padding: const EdgeInsets.only(top: 12),
              child: RoundedButton(
                color: Colors.black.withOpacity(0.3),
                onPressed: () async {
                  if (!moduleContentController.didItemsChange()) {
                    Navigator.pop(context);
                    return;
                  }
                  final itemsByChange = moduleContentController.createItemsByChange();
                  final moduleProvider = Provider.of<ModuleProvider>(context, listen: false)..isInSubmoduleProcess = true;
                  await moduleProvider.startSubmoduleProcess(
                    submoduleAddress: widget.submodule.address,
                    processName: 'fill',
                    itemsByChange: itemsByChange,
                  );
                  if (context.mounted) {
                    final inactivityProvider = Provider.of<InactivityProvider>(context, listen: false);
                    await Navigator.push(context, MaterialPageRoute<ModuleProcessPage>(builder: (context) => const ModuleProcessPage()));
                    inactivityProvider.resetInactivityTimer();
                  }
                  Future.delayed(const Duration(seconds: 1), () => moduleProvider.isInSubmoduleProcess = false);
                  if (context.mounted) {
                    Navigator.pop(context);
                  }
                },
                child: const Padding(
                  padding: EdgeInsets.symmetric(vertical: 32, horizontal: 16),
                  child: Text(
                    'Bestätigen',
                    style: TextStyle(fontSize: 32, color: RobotColors.primaryText),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
