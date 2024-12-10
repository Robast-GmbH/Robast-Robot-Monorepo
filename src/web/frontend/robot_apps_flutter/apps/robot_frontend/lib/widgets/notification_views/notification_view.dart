import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/hygiene_provider.dart';
import 'package:robot_frontend/models/provider/notification_provider.dart';
import 'package:robot_frontend/pages/setting_pages/cleaning_page.dart';
import 'package:robot_frontend/widgets/notification_tile.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class NotificationView extends StatelessWidget {
  const NotificationView({super.key});

  @override
  Widget build(BuildContext context) {
    final hygieneProvider = Provider.of<HygieneProvider>(context, listen: true);
    final notificationProvider = Provider.of<NotificationProvider>(context, listen: true);
    return RoundedContainer(
      child: SizedBox(
        width: double.infinity,
        height: double.infinity,
        child: Padding(
          padding: const EdgeInsets.only(left: 16, right: 16, bottom: 16),
          child: ListView(
            children: [
              if (hygieneProvider.requiresCleaning ?? false)
                Padding(
                  padding: const EdgeInsets.only(top: 16),
                  child: NotificationTile(
                    title: 'Reinigung erforderlich',
                    message: 'Bitte reinigen Sie den Roboter',
                    onPressed: () {
                      Navigator.push(context, MaterialPageRoute<void>(builder: (context) => const CleaningPage()));
                    },
                    actionIcon: Icons.arrow_right_alt,
                  ),
                ),
              if (notificationProvider.heartbeatTimeoutErrors.isNotEmpty)
                ...notificationProvider.heartbeatTimeoutErrors.map(
                  (error) => Padding(
                    padding: const EdgeInsets.only(top: 16),
                    child: NotificationTile(
                      title: 'Heartbeat Timeout Error',
                      message: 'Error code: $error',
                      onPressed: () => notificationProvider.removeHearbeatTimeoutError(error),
                      actionIcon: Icons.clear,
                    ),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }
}
