import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/modules_provider.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class ModuleFillingPage extends StatefulWidget {
  const ModuleFillingPage({super.key});

  @override
  State<ModuleFillingPage> createState() => _ModuleFillingPageState();
}

class _ModuleFillingPageState extends State<ModuleFillingPage> {
  final selectedDrawerAddress = <int>[];
  final payload = <String, int>{};

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
                if (selectedDrawerAddress.isEmpty) {
                  return buildModulesOverview(modules);
                } else {
                  final module = modules.firstWhere(
                    (element) => element.moduleID == selectedDrawerAddress[0] && element.drawerID == selectedDrawerAddress[1],
                  );
                  return buildModuleContentUpdateView(module);
                }
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
                      .map(
                        (e) => Card(
                          color: Colors.white.withOpacity(0.5),
                          child: Padding(
                            padding: const EdgeInsets.all(16),
                            child: Row(
                              mainAxisAlignment: MainAxisAlignment.spaceBetween,
                              children: [
                                _buildListTileText(e.key),
                                Row(
                                  children: [
                                    IconButton(
                                      onPressed: () {
                                        if (payload.containsKey(e.key)) {
                                          setState(() {
                                            payload[e.key] = payload[e.key]! - 1;
                                          });
                                        }
                                      },
                                      icon: const Icon(Icons.remove),
                                    ),
                                    _buildListTileText((e.value + (payload.containsKey(e.key) ? payload[e.key]! : 0)).toString()),
                                    IconButton(
                                      onPressed: () {
                                        if (payload.containsKey(e.key)) {
                                          setState(() {
                                            payload[e.key] = payload[e.key]! + 1;
                                          });
                                        } else {
                                          setState(() {
                                            payload[e.key] = 1;
                                          });
                                        }
                                      },
                                      icon: const Icon(Icons.add),
                                    ),
                                  ],
                                ),
                              ],
                            ),
                          ),
                        ),
                      )
                      .toList() +
                  [
                    Card(
                      color: Colors.white.withOpacity(0.7),
                      child: IconButton(
                        color: Colors.white,
                        iconSize: 64,
                        onPressed: () {},
                        icon: const Icon(Icons.add),
                      ),
                    ),
                  ],
            ),
          ),
          TextButton(
            onPressed: () {
              setState(selectedDrawerAddress.clear);
            },
            child: const Text('Zurück'),
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
                    selectedDrawerAddress
                      ..clear()
                      ..add(e.moduleID)
                      ..add(e.drawerID);
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

  Text _buildListTileText(String text) {
    return Text(
      text,
      style: const TextStyle(fontSize: 24),
    );
  }
}
