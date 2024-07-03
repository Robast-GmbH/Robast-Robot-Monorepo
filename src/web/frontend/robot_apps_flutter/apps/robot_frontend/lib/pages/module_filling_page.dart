import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/pages/module_process_page.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class ModuleFillingPage extends StatefulWidget {
  const ModuleFillingPage({super.key});

  @override
  State<ModuleFillingPage> createState() => _ModuleFillingPageState();
}

class _ModuleFillingPageState extends State<ModuleFillingPage> {
  DrawerAddress? selectedDrawerAddress;
  final payloadDifferences = <String, int>{};
  final createdPayloads = <String, int>{};
  final textController = TextEditingController();
  final amountController = TextEditingController();
  bool isCreatingNewPayload = false;

  @override
  void initState() {
    super.initState();
    Provider.of<ModulesProvider>(context, listen: false).startModulesUpdateTimer();
  }

  @override
  void deactivate() {
    Provider.of<ModulesProvider>(context, listen: false).stopModulesUpdateTimer();
    super.deactivate();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Module auffüllen/entleeren',
      child: Column(
        children: [
          Expanded(
            child: Selector<ModulesProvider, List<RobotDrawer>>(
              selector: (context, provider) => provider.modules,
              builder: (context, modules, child) {
                if (selectedDrawerAddress == null) {
                  return buildModulesOverview(modules);
                } else if (modules.isNotEmpty) {
                  final module = modules.firstWhere(
                    (element) => element.moduleID == selectedDrawerAddress?.moduleID && element.drawerID == selectedDrawerAddress?.drawerID,
                  );
                  return buildModuleContentUpdateView(module);
                }
                return const Center(child: CircularProgressIndicator());
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget buildModuleContentUpdateView(RobotDrawer module) {
    return Padding(
      padding: const EdgeInsets.all(128),
      child: Column(
        children: [
          Expanded(
            child: ListView(
              children: module.content.entries
                      .map<Widget>(
                        buildContentListTile,
                      )
                      .toList() +
                  createdPayloads.entries.map(buildContentListTile).toList() +
                  [
                    buildPayloadCreationView(),
                    Padding(
                      padding: const EdgeInsets.only(bottom: 32),
                      child: Card(
                        color: Colors.white.withOpacity(0.7),
                        child: IconButton(
                          color: Colors.white,
                          iconSize: 64,
                          onPressed: () {
                            setState(() {
                              createdPayloads[textController.text] = int.tryParse(amountController.text) ?? 0;
                              textController.clear();
                              amountController.clear();
                            });
                          },
                          icon: const Icon(Icons.add),
                        ),
                      ),
                    ),
                  ],
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
                  createdPayloads.clear();
                  payloadDifferences.clear();
                  selectedDrawerAddress = null;
                  setState(() {});
                },
                child: const Text('Abbrechen'),
              ),
              TextButton(
                onPressed: () {
                  final payload = <String, int>{}
                    ..addAll(payloadDifferences)
                    ..addAll(createdPayloads);
                  Provider.of<ModulesProvider>(context, listen: false).startModuleProcess(
                    drawerAddress: selectedDrawerAddress!,
                    processName: 'fill',
                    payload: payload,
                  );
                  Navigator.push(
                    context,
                    MaterialPageRoute<ModuleProcessPage>(
                      builder: (context) => const ModuleProcessPage(),
                    ),
                  );
                },
                child: const Text('Bestätigen'),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Card buildContentListTile(MapEntry<String, int> entry) {
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Padding(
            padding: const EdgeInsets.all(16),
            child: _buildListTileText(entry.key),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Row(
              children: [
                IconButton(
                  iconSize: 32,
                  onPressed: () {
                    if (payloadDifferences.containsKey(entry.key)) {
                      setState(() {
                        payloadDifferences[entry.key] = payloadDifferences[entry.key]! - 1;
                      });
                    } else {
                      setState(() {
                        payloadDifferences[entry.key] = -1;
                      });
                    }
                  },
                  icon: const Icon(Icons.remove),
                ),
                _buildListTileText(
                  (entry.value + (payloadDifferences.containsKey(entry.key) ? payloadDifferences[entry.key]! : 0)).toString(),
                ),
                IconButton(
                  iconSize: 32,
                  onPressed: () {
                    if (payloadDifferences.containsKey(entry.key)) {
                      setState(() {
                        payloadDifferences[entry.key] = payloadDifferences[entry.key]! + 1;
                      });
                    } else {
                      setState(() {
                        payloadDifferences[entry.key] = 1;
                      });
                    }
                  },
                  icon: const Icon(Icons.add),
                ),
              ],
            ),
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
              (e) => GestureDetector(
                onTap: () {
                  setState(() {
                    selectedDrawerAddress = DrawerAddress(
                      moduleID: e.moduleID,
                      drawerID: e.drawerID,
                    );
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
                        _buildListTileText(
                          '${e.moduleID}_${e.drawerID}',
                        ),
                        _buildListTileText(
                          e.content.entries.map((e) => ' ${e.key}: ${e.value}').fold('', (previousValue, element) => previousValue + element),
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

  Card buildPayloadCreationView() {
    return Card(
      color: Colors.white.withOpacity(0.5),
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Expanded(
              flex: 8,
              child: TextField(
                controller: textController,
                style: const TextStyle(fontSize: 24),
              ),
            ),
            const SizedBox(
              width: 32,
            ),
            Expanded(
              child: TextField(
                textAlign: TextAlign.center,
                controller: amountController,
                style: const TextStyle(fontSize: 24),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Text _buildListTileText(String text) {
    return Text(
      text,
      style: const TextStyle(fontSize: 24),
    );
  }
}
