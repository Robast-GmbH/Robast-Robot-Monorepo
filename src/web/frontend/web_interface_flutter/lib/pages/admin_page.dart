import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:web_interface_flutter/models/admin_menu_point.dart';
import 'package:web_interface_flutter/pages/admin_sub_page.dart';
import 'package:web_interface_flutter/widgets/api_address_input_field.dart';
import 'package:web_interface_flutter/widgets/docker_overview.dart';
import 'package:web_interface_flutter/widgets/patrol_overview.dart';
import 'package:web_interface_flutter/widgets/resume_button.dart';
import 'package:web_interface_flutter/widgets/robot_labeling_overview.dart';

import 'package:web_interface_flutter/widgets/script_list.dart';
import 'package:web_interface_flutter/widgets/tasks_overview.dart';

class AdminPage extends StatefulWidget {
  const AdminPage({super.key});

  @override
  State<AdminPage> createState() => _AdminPageState();
}

class _AdminPageState extends State<AdminPage> {
  final adminMenuPoints = [
    AdminMenuPoint(title: "docker", icon: Icons.storage_rounded, widget: const DockerOverview()),
    AdminMenuPoint(title: "tasks", icon: Icons.task, widget: const TasksOverview()),
    AdminMenuPoint(title: "API", icon: Icons.web, widget: const APIAddressInputField()),
    AdminMenuPoint(title: "scripts", icon: Icons.developer_board, widget: const ScriptList()),
    AdminMenuPoint(title: "modules", icon: Icons.draw, widget: const RobotLabelingOverview()),
    AdminMenuPoint(title: "patrol", icon: Icons.circle_outlined, widget: const PatrolOverview()),
    AdminMenuPoint(title: "resume", icon: Icons.play_arrow, widget: const ResumeButton())
  ];

  int selectedIndex = 0;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("Admin"),
      ),
      body: Row(
        children: [
          Expanded(
            child: Container(
              decoration: const BoxDecoration(
                color: Colors.white,
                boxShadow: [
                  BoxShadow(
                    color: Colors.grey,
                    offset: Offset(0, 1),
                    blurRadius: 2,
                    spreadRadius: 1,
                  ),
                ],
              ),
              child: ListView(
                children: ListTile.divideTiles(
                  context: context,
                  tiles: List.generate(
                    adminMenuPoints.length,
                    (index) => ListTile(
                      leading: Icon(adminMenuPoints[index].icon),
                      selected: !kIsWeb && selectedIndex == index,
                      title: Text(adminMenuPoints[index].title),
                      onTap: () {
                        if (kIsWeb) {
                          Navigator.push(
                            context,
                            MaterialPageRoute(
                              builder: (context) => AdminSubPage(
                                title: adminMenuPoints[index].title,
                                child: adminMenuPoints[index].widget,
                              ),
                            ),
                          );
                          return;
                        }
                        selectedIndex = index;
                        setState(() {});
                      },
                    ),
                  ),
                ).toList(),
              ),
            ),
          ),
          if (!kIsWeb)
            Expanded(
              flex: 5,
              child: adminMenuPoints[selectedIndex].widget,
            ),
        ],
      ),
    );
  }
}
