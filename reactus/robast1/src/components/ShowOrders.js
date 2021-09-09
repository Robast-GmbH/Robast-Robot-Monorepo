import PropTypes from 'prop-types'
import Orders from "./Orders"


const ShowOrders = ({ tasks: orders, onDelete, onToggle }) => {
        return (
                <div>
                        {orders > 0 ? (
                                <Orders
                                        orders={orders}
                                        onDelete={onDelete}
                                        onToggle={onToggle}>
                                </Orders>
                        ) : (
                                'empty'
                        )}
                </div>
        )
}


ShowOrders.protoTypes = {
        orders: PropTypes.object,
        onDelete: PropTypes.func,
        onToggle: PropTypes.func,
}

export default ShowOrders
