import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/data/svgs.dart';
import 'package:robot_frontend/pages/caire_ai_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class CaireAiCardView extends StatelessWidget {
  const CaireAiCardView({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      onPressed: () {
        Navigator.push(context, MaterialPageRoute(builder: (context) => CaireAiPage()));
      },
      content: Center(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                SvgPicture.string(
                  logo,
                  colorFilter: const ColorFilter.mode(
                    RobotColors.primaryIcon,
                    BlendMode.srcIn,
                  ),
                  height: 256,
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 64),
                  child: Container(
                    width: 2,
                    height: 200,
                    color: Colors.white.withOpacity(0.7),
                  ),
                ),
                Image.asset(
                  'assets/cairelogo.jpeg',
                  height: 256,
                ),
              ],
            ),
            SizedBox(
              height: 48,
            ),
            const Text('Hier drücken für caire.ai',
                style: TextStyle(
                  color: RobotColors.primaryText,
                  fontSize: 80,
                  fontWeight: FontWeight.bold,
                )),
          ],
        ),
      ),
    );
  }
}
