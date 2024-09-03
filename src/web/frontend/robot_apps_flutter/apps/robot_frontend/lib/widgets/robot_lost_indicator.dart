import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/robot_localization_page.dart';

class RobotLostIndicator extends StatefulWidget {
  const RobotLostIndicator({super.key});

  @override
  State<RobotLostIndicator> createState() => _RobotLostIndicatorState();
}

class _RobotLostIndicatorState extends State<RobotLostIndicator> {
  @override
  void initState() {
    super.initState();
    Provider.of<RobotProvider>(context, listen: false).startPeriodicIsRobotLostUpdate();
  }

  @override
  void dispose() {
    Provider.of<RobotProvider>(context, listen: false).stopPeriodicIsRobotLostUpdate();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        if (Provider.of<RobotProvider>(context, listen: false).isRobotLost) {
          Navigator.push(context, MaterialPageRoute<RobotLocalizationPage>(builder: (context) => const RobotLocalizationPage()));
        }
      },
      child: Container(
        margin: const EdgeInsets.all(4),
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: Provider.of<RobotProvider>(context).isRobotLost ? const Color.fromARGB(255, 226, 34, 0) : const Color.fromARGB(255, 0, 226, 0),
          border: Border.all(
            color: RobotColors.primaryText,
            width: 4,
          ),
        ),
        width: 30,
        height: 30,
      ),
    );
  }
}
