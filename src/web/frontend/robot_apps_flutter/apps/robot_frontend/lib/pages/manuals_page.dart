import 'package:flutter/material.dart';
import 'package:robot_frontend/pages/pdf_page.dart';
import 'package:robot_frontend/services/manuals_manager.dart';
import 'package:robot_frontend/widgets/buttons/custom_elevated_button.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class ManualsPage extends StatefulWidget {
  const ManualsPage({this.inactivityTimerEnabled = true, super.key});
  final bool inactivityTimerEnabled;
  @override
  State<ManualsPage> createState() => _ManualsPageState();
}

class _ManualsPageState extends State<ManualsPage> {
  final manualsManager = ManualsManager();
  final manuals = <String>[];
  late final Future<void> _updateManuals;

  Future<void> getManuals() async {
    await manualsManager.updateManuals();
    final manualsList = await manualsManager.getManualsList();
    manuals.addAll(manualsList);
  }

  @override
  void initState() {
    super.initState();
    _updateManuals = getManuals();
  }

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      inactivityTimerEnabled: widget.inactivityTimerEnabled,
      title: 'Anleitungen',
      child: FutureBuilder<void>(
          future: _updateManuals,
          builder: (context, snapshot) {
            if (snapshot.connectionState != ConnectionState.done) {
              return const Center(child: CircularProgressIndicator());
            }
            return ListView(
              children: List.generate(
                manuals.length,
                (index) => Padding(
                  padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 64),
                  child: CustomElevatedButton(
                    label: manuals[index],
                    onPressed: () async {
                      final baseDir = await manualsManager.getCacheDir();
                      Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => PdfPage(
                            path: '$baseDir/${manuals[index]}',
                            inactivityTimerEnabled: widget.inactivityTimerEnabled,
                          ),
                        ),
                      );
                    },
                  ),
                ),
              ),
            );
          }),
    );
  }
}
