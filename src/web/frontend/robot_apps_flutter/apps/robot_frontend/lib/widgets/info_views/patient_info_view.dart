import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';

class PatientInfoView extends StatelessWidget {
  const PatientInfoView({super.key});

  final videoPath = 'assets/robast_video.mp4';

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
                        errorBuilder: (context, error, stackTrace) => const Icon(Icons.group, size: 100, color: RobotColors.primaryText),
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
                  header: const Text(
                    'Anleitungen',
                    style: TextStyle(color: RobotColors.primaryText, fontSize: 40, fontWeight: FontWeight.w400),
                  ),
                  onPressed: () {
                    Navigator.push(context, MaterialPageRoute(builder: (context) => const ManualsPage()));
                  },
                  content: const Center(
                    child: Icon(
                      Icons.book,
                      size: 100,
                      color: RobotColors.primaryText,
                    ),
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
