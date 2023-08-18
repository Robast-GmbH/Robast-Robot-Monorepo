import 'package:flutter/material.dart';
import 'package:intl/date_symbol_data_local.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/models/robot_provider.dart';
import 'package:web_interface_flutter/pages/accept_order_page.dart';
import 'package:web_interface_flutter/pages/admin_page.dart';
import 'package:web_interface_flutter/pages/login_page.dart';
import 'package:web_interface_flutter/pages/manual_control_page.dart';
import 'package:web_interface_flutter/pages/task_creation_page.dart';
import 'package:web_interface_flutter/widgets/battery_level_indicator.dart';
import 'package:web_interface_flutter/widgets/rounded_button.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  late Stream<DateTime> dateTimeStream;
  late Future<void> loadData;
  @override
  void initState() {
    super.initState();
    loadData = Provider.of<RobotProvider>(context, listen: false).updateProviderData();
    dateTimeStream = Stream.periodic(const Duration(seconds: 1), (count) {
      return DateTime.now();
    }).asBroadcastStream();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: AppColors.grey,
      body: Stack(
        children: [
          Row(
            children: [
              const Expanded(child: SizedBox()),
              Expanded(
                flex: 2,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    Column(
                      children: [
                        Container(
                          padding: const EdgeInsets.all(8) + const EdgeInsets.only(top: 8),
                          margin: const EdgeInsets.all(8),
                          decoration: BoxDecoration(borderRadius: BorderRadius.circular(32), color: AppColors.darkBlue),
                          child: Column(
                            children: [
                              Column(
                                children: [
                                  Row(
                                    children: [
                                      const SizedBox(
                                        width: 16,
                                      ),
                                      Expanded(
                                        flex: 2,
                                        child: FutureBuilder<void>(
                                          future: initializeDateFormatting("de_DE"),
                                          builder: (context, snapshot) {
                                            if (snapshot.connectionState == ConnectionState.done) {
                                              return StreamBuilder<DateTime>(
                                                stream: dateTimeStream,
                                                builder: (context, stream) {
                                                  final date = stream.hasData ? stream.data! : DateTime.now();
                                                  return Row(
                                                    children: [
                                                      Expanded(
                                                          child: Text(
                                                        DateFormat('d. MMMM y').format(date),
                                                        style: const TextStyle(color: AppColors.white, fontSize: 12, fontWeight: FontWeight.w300),
                                                      )),
                                                      Expanded(
                                                        child: Text(
                                                          "${date.hour}:${date.minute < 10 ? "0${date.minute}" : date.minute}",
                                                          textAlign: TextAlign.center,
                                                          style: const TextStyle(color: AppColors.white, fontSize: 12, fontWeight: FontWeight.w300),
                                                        ),
                                                      ),
                                                    ],
                                                  );
                                                },
                                              );
                                            }
                                            return const SizedBox();
                                          },
                                        ),
                                      ),
                                      Expanded(
                                        child: Row(
                                          children: [
                                            const Expanded(child: SizedBox()),
                                            Container(
                                              padding: const EdgeInsets.symmetric(vertical: 2, horizontal: 4),
                                              decoration: BoxDecoration(
                                                borderRadius: BorderRadius.circular(4),
                                                color: AppColors.darkBlueAccent,
                                              ),
                                              child: const BatteryLevelIndicator(),
                                            ),
                                            const SizedBox(
                                              width: 16,
                                            ),
                                          ],
                                        ),
                                      ),
                                    ],
                                  ),
                                  const SizedBox(
                                    height: 16,
                                  ),
                                  const Row(
                                    mainAxisAlignment: MainAxisAlignment.center,
                                    children: [
                                      Text("Hey, ich bin", style: TextStyle(color: Colors.white, fontSize: 32, fontWeight: FontWeight.w300)),
                                      Text(" Rosalie", style: TextStyle(color: Colors.white, fontSize: 32, fontWeight: FontWeight.w500)),
                                    ],
                                  ),
                                  const SizedBox(
                                    height: 4,
                                  ),
                                  const Text(
                                    "Ein Roboter von Robast Robotic Assistant GmbH",
                                    style: TextStyle(color: AppColors.white, fontSize: 10, fontWeight: FontWeight.w400),
                                  ),
                                  const SizedBox(
                                    height: 8,
                                  ),
                                ],
                              ),
                              Row(
                                children: [
                                  buildInfoBox(title: "Error Log:", content: 'Multiple "Network"', fontSize: 9, borderRadius: BorderRadius.circular(16)),
                                  buildInfoBox(
                                      title: "Aktueller Auftrag:", content: 'Ausliefern F02', fontSize: 20, borderRadius: BorderRadius.circular(24), flex: 5),
                                  buildInfoBox(title: "Nächster Auftrag:", content: 'Ausliefern F03', fontSize: 9, borderRadius: BorderRadius.circular(16)),
                                ],
                              )
                            ],
                          ),
                        ),
                      ],
                    ),
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: RoundedButton(
                        text: "Freie Schubladenauswahl",
                        color: AppColors.blue,
                        borderRadius: BorderRadius.circular(32),
                        onTap: () => Navigator.push(
                          context,
                          MaterialPageRoute(
                            builder: (context) => const ManualControlPage(),
                          ),
                        ),
                      ),
                    ),
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: RoundedButton(
                        text: "Lieferung annehmen",
                        color: AppColors.turquoise,
                        borderRadius: BorderRadius.circular(32),
                        onTap: () => Navigator.push(context, MaterialPageRoute(builder: (context) => AcceptOrderPage())),
                      ),
                    ),
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: RoundedButton(
                        text: "Neuer Auftrag",
                        color: AppColors.green,
                        borderRadius: BorderRadius.circular(32),
                        onTap: () => Navigator.push(context, MaterialPageRoute(builder: (context) => const TaskCreationPage())),
                      ),
                    ),
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: RoundedButton(
                        text: "Einstellungen",
                        color: AppColors.white,
                        borderRadius: BorderRadius.circular(32),
                        textColor: Colors.black,
                      ),
                    ),
                  ],
                ),
              ),
              const Expanded(child: SizedBox())
            ],
          ),
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                IconButton(
                  onPressed: () {
                    Navigator.pushReplacement(
                      context,
                      MaterialPageRoute(
                        builder: (context) => const LoginPage(),
                      ),
                    );
                  },
                  icon: const Icon(Icons.account_circle),
                  color: AppColors.white,
                  iconSize: 32,
                ),
                IconButton(
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (context) => const AdminPage(),
                      ),
                    );
                  },
                  icon: const Icon(Icons.settings),
                  color: AppColors.white,
                  iconSize: 32,
                ),
              ],
            ),
          )
        ],
      ),
    );
  }

  Widget buildInfoBox({
    required String title,
    required String content,
    int flex = 1,
    required double fontSize,
    required BorderRadius borderRadius,
  }) {
    return Expanded(
      flex: flex,
      child: Container(
        height: 128,
        margin: const EdgeInsets.all(8),
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
          borderRadius: borderRadius,
          color: AppColors.darkBlueAccent,
        ),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Text(
              title,
              style: TextStyle(
                color: AppColors.white,
                fontSize: fontSize,
              ),
            ),
            const SizedBox(
              height: 8,
            ),
            Text(
              content,
              style: TextStyle(
                color: AppColors.white,
                fontSize: fontSize,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
