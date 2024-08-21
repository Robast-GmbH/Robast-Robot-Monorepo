import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_button_view.dart';
import 'package:robot_frontend/widgets/rounded_button.dart';

class PatientHomeView extends StatelessWidget {
  const PatientHomeView({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          flex: 2,
          child: CustomButtonView(
            text: "Hallo Nutzer:in,",
            content: Text('mein Name ist Rosalina.', style: const TextStyle(color: Colors.white70, fontSize: 24)),
            onPressed: () {},
          ),
        ),
        SizedBox(width: 16),
        Expanded(
          child: Column(
            children: [
              Expanded(
                  child: CustomButtonView(
                text: 'Lieferplan',
                onPressed: () {},
                content: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Expanded(child: Text('Essen:', style: const TextStyle(color: Colors.white70, fontSize: 24))),
                        Expanded(child: Text('Lasagne', textAlign: TextAlign.start, style: const TextStyle(color: Colors.white70, fontSize: 24))),
                        Expanded(child: Text('17:00 Uhr', textAlign: TextAlign.end, style: const TextStyle(color: Colors.white70, fontSize: 24))),
                      ],
                    ),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Expanded(child: Text('Medikament:', style: const TextStyle(color: Colors.white70, fontSize: 24))),
                        Expanded(child: Text('Aspirin', textAlign: TextAlign.start, style: const TextStyle(color: Colors.white70, fontSize: 24))),
                        Expanded(child: Text('19:00 Uhr', textAlign: TextAlign.end, style: const TextStyle(color: Colors.white70, fontSize: 24))),
                      ],
                    ),
                  ],
                ),
              )),
              SizedBox(height: 16),
              Expanded(
                  child: CustomButtonView(
                text: 'Aktivitäten',
                onPressed: () {},
                content: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    SizedBox(height: 16),
                    Expanded(
                      child: RoundedButton(
                        onPressed: () {},
                        child: Text(
                          "Essen bestellen",
                          style: TextStyle(fontSize: 32, color: Colors.white70),
                        ),
                        color: Colors.black.withOpacity(0.1),
                      ),
                    ),
                    SizedBox(height: 16),
                    Expanded(
                      child: RoundedButton(
                        onPressed: () {},
                        child: Text(
                          "Getränke bestellen",
                          style: TextStyle(fontSize: 32, color: Colors.white70),
                        ),
                        color: Colors.black.withOpacity(0.1),
                      ),
                    ),
                    SizedBox(height: 16),
                    Expanded(
                      child: RoundedButton(
                        onPressed: () {},
                        child: Text(
                          "Pflegepersonal rufen",
                          style: TextStyle(fontSize: 32, color: Colors.white70),
                        ),
                        color: Colors.red.withOpacity(0.4),
                      ),
                    )
                  ],
                ),
              )),
            ],
          ),
        )
      ],
    );
  }
}
