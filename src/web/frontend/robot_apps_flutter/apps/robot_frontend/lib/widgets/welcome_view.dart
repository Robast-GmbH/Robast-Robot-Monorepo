import 'package:flutter/material.dart';
import 'package:flutter_svg/svg.dart';
import 'package:robot_frontend/data/svgs.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';

class WelcomeView extends StatelessWidget {
  const WelcomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomButtonView(
      text: "Hallo Nutzer:in,",
      content: Text('mein Name ist Rosalina.', style: const TextStyle(color: Colors.white70, fontSize: 24)),
      onPressed: () {},
      trailing: Padding(
        padding: const EdgeInsets.all(16),
        child: SvgPicture.string(logo, color: Colors.white, height: 128),
      ),
    );
  }
}
