import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/pages/video_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/video_view.dart';

class InfoView extends StatelessWidget {
  const InfoView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: CustomButtonView(
                  text: "Robast Robotic Assistant GmbH",
                  onPressed: () {},
                  content: const Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      SizedBox(height: 8),
                      InfoRow(
                        title: "Ziel",
                        value: "Entwicklung eines autonomen Serviceroboters für die Prozessautomatisierung im Krankenhaus",
                      ),
                      InfoRow(title: "Gründung", value: "2021"),
                      InfoRow(title: "Teamgröße", value: "7 Personen"),
                      InfoRow(title: "Standort", value: "Hamburg Harburg"),
                      InfoRow(
                        title: "Partner",
                        value: "Tiplu GmbH, IFB Hamburg, Fraunhofer IAPT",
                      ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Unser Team',
                  onPressed: () {},
                  content: Padding(
                    padding: const EdgeInsets.only(top: 4),
                    child: Center(
                        child: ClipRRect(
                      borderRadius: BorderRadius.circular(8),
                      child: Image.asset(
                        'assets/team.jpeg',
                        fit: BoxFit.fitWidth,
                        width: double.infinity,
                      ),
                    )),
                  ),
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 16),
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: CustomButtonView(
                  onPressed: () {
                    Navigator.push(context, MaterialPageRoute<VideoPage>(builder: (context) => const VideoPage()));
                  },
                  content: GestureDetector(
                    behavior: HitTestBehavior.opaque,
                    child: AbsorbPointer(
                        absorbing: true,
                        child: Stack(
                          children: [
                            ClipRRect(borderRadius: BorderRadius.circular(8), child: const VideoView()),
                            const Align(
                                alignment: Alignment.bottomRight,
                                child: Padding(
                                  padding: EdgeInsets.all(8.0),
                                  child: Icon(Icons.fullscreen, size: 48, color: RobotColors.primaryText),
                                )),
                          ],
                        )),
                    onTap: () {
                      Navigator.push(context, MaterialPageRoute<VideoPage>(builder: (context) => const VideoPage()));
                    },
                  ),
                ),
              ),
              const SizedBox(height: 16),
              Expanded(
                child: CustomButtonView(
                  text: 'Kontakt',
                  onPressed: () {},
                  content: Row(
                    children: [
                      const Expanded(
                        child: Padding(
                          padding: EdgeInsets.only(left: 4, top: 16),
                          child: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text('Sie haben Fragen oder wollen mehr über Robast erfahren?', style: TextStyle(color: RobotColors.secondaryText, fontSize: 24)),
                              SizedBox(height: 32),
                              Text('Wir freuen uns auf Ihre Nachricht:', style: TextStyle(color: RobotColors.secondaryText, fontSize: 24)),
                              Text('info@robast.de', style: TextStyle(color: RobotColors.secondaryText, fontSize: 28, fontWeight: FontWeight.bold)),
                              SizedBox(height: 16),
                              Text('Besuchen Sie unsere Website:', style: TextStyle(color: RobotColors.secondaryText, fontSize: 24)),
                              Text('www.robast.de', style: TextStyle(color: RobotColors.secondaryText, fontSize: 28, fontWeight: FontWeight.bold)),
                            ],
                          ),
                        ),
                      ),
                      Expanded(
                        child: Center(
                          child: Padding(
                            padding: const EdgeInsets.only(bottom: 64, right: 32, top: 16),
                            child: Image.asset(
                              'assets/white_qrcode.png',
                              alignment: Alignment.centerRight,
                            ),
                          ),
                        ),
                      )
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

class InfoRow extends StatelessWidget {
  final String title;
  final String value;

  const InfoRow({
    super.key,
    required this.title,
    required this.value,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            "$title: ",
            style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 24, color: RobotColors.secondaryText),
          ),
          Expanded(
            child: Text(
              value,
              style: const TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
          ),
        ],
      ),
    );
  }
}
