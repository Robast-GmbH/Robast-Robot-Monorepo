import 'package:flutter/material.dart';
import 'package:web_interface_flutter/widgets/script_button.dart';

class ScriptList extends StatefulWidget {
  const ScriptList({super.key});

  @override
  State<ScriptList> createState() => _ScriptListState();
}

class _ScriptListState extends State<ScriptList> {
  final List<List<String>> arguments = [
    ['/home/robot/robast/scripts/shutdown_jetson.sh'],
    ['ssh robast@192.168.0.50 "docker restart robo_a_statemachine_1"'],
  ];

  final List<String> titles = ["Nano ausschalten", "Robo_A_Statemachine_1 neustarten"];

  @override
  Widget build(BuildContext context) {
    return Column(
      children: List.generate(
        titles.length,
        (index) => ScriptButton(
          arguments: arguments[index],
          title: titles[index],
        ),
      ),
    );
  }
}
