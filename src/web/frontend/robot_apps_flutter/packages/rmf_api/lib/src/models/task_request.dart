class PatrolTaskRequest extends TaskRequest {
  PatrolTaskRequest({
    required List<String> places,
    required int rounds,
  }) : super(
          category: 'patrol',
          description: {
            'rounds': rounds,
            'places': places,
          },
        );
}

class DeliveryTaskRequest extends TaskRequest {
  DeliveryTaskRequest({
    required String dropoff,
    required String pickup,
    required String drawerID,
  }) : super(
          category: 'delivery',
          description: {
            'dropoff': {
              'handler': 'drawer_ingestor',
              'payload': {'quantity': 1, 'sku': drawerID},
              'place': dropoff,
            },
            'pickup': {
              'handler': 'drawer_dispenser',
              'payload': {'quantity': 1, 'sku': drawerID},
              'place': pickup,
            },
          },
        );
}

class TaskRequest {
  TaskRequest({
    required this.category,
    required this.description,
  });

  final String category;
  final Map<String, dynamic> description;

  Map<String, dynamic> toJson() {
    return {
      'request': {
        'category': category,
        'description': description,
        'priority': {'value': 0, 'type': 'binary'},
        'requester': 'dart_package',
        'unix_millis_earliest_start_time': 0,
        'unix_millis_request_time': DateTime.now().millisecondsSinceEpoch,
      },
      'type': 'dispatch_task_request',
    };
  }
}
