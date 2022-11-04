import PropTypes from 'prop-types'
import Orders from "./Orders"


const ShowOrders = ({ orders: orders, onDelete, onToggle }) => {
        return (
                <div>
                        {orders ? (
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
