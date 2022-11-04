import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'

const Order = ({ order: order, onDelete, onToggle }) => {
        return (
                <div className={`task ${order.recurring_order ? 'recurring_order' : ''}`} onDoubleClick={() => onToggle(order.id)}>
                        <h3>
                                {order.order_item}<FaTimes style={{ color: 'red', cursor: 'pointer' }}
                                        onClick={() => onDelete(order.id)}
                                />
                        </h3>
                        {(order.goal.x && order.goal.y)? (

                                <p>{order.goal.x}, {order.goal.y}</p>
                        ):(
                                <p>nothing to see here</p>
                        )}
                </div>
        )
}



Order.propTypes = {
        onDelete: PropTypes.func,
        onToggle: PropTypes.func,
}

export default Order
