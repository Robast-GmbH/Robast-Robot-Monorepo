import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/controller/module_content_controller.dart';
import 'package:robot_frontend/models/provider/module_provider.dart';
import 'package:robot_frontend/pages/module_process_page.dart';
import 'package:robot_frontend/widgets/module_content_creation_view.dart';

class ModuleFillingView extends StatefulWidget {
  const ModuleFillingView({this.preselectedDrawers, super.key});

  final List<DrawerAddress>? preselectedDrawers;

  @override
  State<ModuleFillingView> createState() => _ModuleFillingViewState();
}

class _ModuleFillingViewState extends State<ModuleFillingView> {
  DrawerAddress? selectedDrawerAddress;
  final moduleContentController = ModuleContentController();

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Expanded(
          child: Selector<ModuleProvider, List<RobotDrawer>>(
            selector: (context, provider) => provider.submodules,
            builder: (context, modules, child) {
              if (selectedDrawerAddress == null) {
                return buildModulesOverview(
                  widget.preselectedDrawers == null
                      ? modules
                      : modules.where((drawer) => widget.preselectedDrawers!.any((preselectedDrawer) => preselectedDrawer == drawer.address)).toList(),
                );
              } else if (modules.isNotEmpty) {
                final module = modules.firstWhere(
                  (element) => element.address == selectedDrawerAddress,
                );
                return buildModuleContentUpdateView(module);
              }
              return const Center(child: CircularProgressIndicator());
            },
          ),
        ),
      ],
    );
  }

  Widget buildModuleContentUpdateView(RobotDrawer module) {
    return Padding(
      padding: const EdgeInsets.all(128),
      child: Column(
        children: [
          Expanded(
            child: ModuleContentCreationView(
              moduleContentController: moduleContentController,
            ),
          ),
          const SizedBox(
            height: 16,
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              TextButton(
                onPressed: () {
                  moduleContentController.clear();
                  selectedDrawerAddress = null;
                  setState(() {});
                },
                child: const Text('Abbrechen'),
              ),
              TextButton(
                onPressed: () async {
                  if (moduleContentController.contentItemsByChange.isEmpty && moduleContentController.createdItemsByCount.isEmpty) {
                    return;
                  }
                  final itemsByChange = moduleContentController.createItemsByChange();
                  final moduleProvider = Provider.of<ModuleProvider>(context, listen: false)..isInModuleProcess = true;
                  await moduleProvider.startModuleProcess(
                    drawerAddress: selectedDrawerAddress!,
                    processName: 'fill',
                    itemsByChange: itemsByChange,
                  );
                  if (mounted) {
                    await Navigator.push(context, MaterialPageRoute<ModuleProcessPage>(builder: (context) => const ModuleProcessPage()));
                  }
                  moduleProvider.isInModuleProcess = false;
                  selectedDrawerAddress = null;
                  moduleContentController.clear();
                },
                child: const Text('Bestätigen'),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Padding buildModulesOverview(List<RobotDrawer> modules) {
    return Padding(
      padding: const EdgeInsets.all(128),
      child: ListView(
        children: modules
            .map(
              (drawer) => GestureDetector(
                onTap: () {
                  setState(() {
                    selectedDrawerAddress = drawer.address;
                    moduleContentController.initialItemsByCount.clear();
                    moduleContentController.initialItemsByCount.addAll(drawer.itemsByCount);
                  });
                },
                child: Card(
                  color: Colors.white.withOpacity(0.5),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(16),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        buildListTileText(
                          '${drawer.address.moduleID}_${drawer.address.drawerID}',
                        ),
                        buildListTileText(
                          drawer.itemsByCount.entries.map((e) => ' ${e.key}: ${e.value}').fold('', (previousValue, element) => previousValue + element),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            )
            .toList(),
      ),
    );
  }

  Text buildListTileText(String text) {
    return Text(
      text,
      style: const TextStyle(fontSize: 24),
    );
  }
}
