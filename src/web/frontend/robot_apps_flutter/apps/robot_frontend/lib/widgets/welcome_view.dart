import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/data/svgs.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class WelcomeView extends StatelessWidget {
  const WelcomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      content: Center(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            SvgPicture.string(
              logo,
              colorFilter: const ColorFilter.mode(
                RobotColors.primaryIcon,
                BlendMode.srcIn,
              ),
              height: 128,
            ),
            const Text('Hallo,',
                style: TextStyle(
                  color: RobotColors.primaryText,
                  fontSize: 80,
                  fontWeight: FontWeight.bold,
                )),
            const Text('mein Name ist Robast Cura.',
                style: TextStyle(
                  color: RobotColors.primaryText,
                  fontSize: 32,
                  fontWeight: FontWeight.bold,
                )),
            const SizedBox(height: 8),
            const Text('Ich bin ein Serviceroboter der Robast Robotic Assistant GmbH.',
                style: TextStyle(
                  color: RobotColors.secondaryText,
                  fontSize: 18,
                )),
          ],
        ),
      ),
      onPressed: () {},
    );
  }
}
