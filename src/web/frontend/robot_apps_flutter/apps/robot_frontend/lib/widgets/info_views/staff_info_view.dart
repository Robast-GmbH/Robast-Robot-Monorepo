import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/constants/robot_constants.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/pages/manuals_page.dart';
import 'package:robot_frontend/widgets/buttons/custom_button_view.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class StaffInfoView extends StatefulWidget {
  const StaffInfoView({super.key});

  @override
  State<StaffInfoView> createState() => _StaffInfoViewState();
}

class _StaffInfoViewState extends State<StaffInfoView> {
  late Future<List<String>?> loadErrorLog;

  @override
  void initState() {
    loadErrorLog = Provider.of<RobotProvider>(context, listen: false).getErrorProtocol();
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: Column(
            children: [
              Expanded(
                child: CustomButtonView(
                  onPressed: () {},
                  header: const Text(
                    'Fehlerprotokoll',
                    style: TextStyle(color: RobotColors.primaryText, fontSize: 40, fontWeight: FontWeight.w400),
                  ),
                  content: FutureBuilder(
                      future: loadErrorLog,
                      builder: (context, snapshot) {
                        if (snapshot.connectionState != ConnectionState.done) {
                          return const Center(child: CircularProgressIndicator());
                        }
                        if (snapshot.hasError) {
                          return Center(
                            child: GestureDetector(
                              onTap: () => setState(() {
                                loadErrorLog = Provider.of<RobotProvider>(context, listen: false).getErrorProtocol();
                              }),
                              child: const Column(
                                children: [
                                  Text('Fehler beim Laden des Fehlerprotokolls'),
                                  Text('Erneut versuchen', style: TextStyle(color: RobotColors.secondaryText)),
                                ],
                              ),
                            ),
                          );
                        }
                        return ListView.builder(
                            itemCount: snapshot.data!.length,
                            itemBuilder: (context, index) {
                              final splittedData = snapshot.data![index].split(' - ');
                              final date = splittedData.first;
                              final error = jsonDecode(splittedData.last.split('received: ').last.replaceAll("'", '"')) as Map<String, dynamic>;
                              return Padding(
                                padding: const EdgeInsets.symmetric(vertical: 8),
                                child: RoundedContainer(
                                  child: Padding(
                                    padding: const EdgeInsets.all(8.0),
                                    child: Column(
                                      crossAxisAlignment: CrossAxisAlignment.start,
                                      children: [
                                        Text(
                                          date,
                                          style: const TextStyle(color: RobotColors.secondaryText, fontSize: 28),
                                        ),
                                        ...error.entries.map((entry) {
                                          return InfoRow(title: entry.key, value: entry.value.toString());
                                        }),
                                      ],
                                    ),
                                  ),
                                ),
                              );
                            });
                      }),
                ),
              )
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
