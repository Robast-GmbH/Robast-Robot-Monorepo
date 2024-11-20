import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';

class DisinfectionModuleEmptyView extends StatelessWidget {
  const DisinfectionModuleEmptyView({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        const Text(
          "Desinfektionsmittel leer",
          style: TextStyle(
            fontSize: 80,
            color: RobotColors.primaryText,
          ),
        ),
        const SizedBox(height: 32),
        ElevatedButton(
          onPressed: () async {
            final robotProvider = Provider.of<RobotProvider>(context, listen: false);
            await robotProvider.renewDisinfectionFluidContainer();
            await robotProvider.updateRemainingDisinfections();
          },
          child: const Padding(
            padding: EdgeInsets.symmetric(horizontal: 32),
            child: Text(
              "Auffüllen",
              style: TextStyle(
                fontSize: 80,
                color: RobotColors.primaryText,
              ),
            ),
          ),
        )
      ],
    );
  }
}
